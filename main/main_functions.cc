#include "main_functions.h"

#include "detection_responder.h"
#include "image_provider.h"
#include "model_settings.h"
#include "person_detect_model_data.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_log.h>
#include "esp_main.h"

namespace {
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;

#if CONFIG_NN_OPTIMIZED
constexpr int scratchBufSize = 60 * 1024;
#else
constexpr int scratchBufSize = 0;
#endif

constexpr int kTensorArenaSize = 100 * 1024 + scratchBufSize;
static uint8_t *tensor_arena;

// ROI detection: number of ROIs to process per frame
constexpr int NUM_ROIS = 4;
}  // namespace

void setup() {
  model = tflite::GetModel(g_person_detect_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model schema version %d not equal to supported version %d.",
                model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  if (tensor_arena == NULL) {
    tensor_arena = (uint8_t *) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (tensor_arena == NULL) {
    MicroPrintf("Couldn't allocate memory of %d bytes\n", kTensorArenaSize);
    return;
  }

  // Original 5 operations for 96×96 grayscale model
  static tflite::MicroMutableOpResolver<5> micro_op_resolver;
  micro_op_resolver.AddAveragePool2D();
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddDepthwiseConv2D();
  micro_op_resolver.AddReshape();
  micro_op_resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  input = interpreter->input(0);

#if !CLI_ONLY_INFERENCE
  TfLiteStatus init_status = InitCamera();
  if (init_status != kTfLiteOk) {
    MicroPrintf("InitCamera failed\n");
    return;
  }

#if DISPLAY_SUPPORT
  create_gui();
#endif
#endif

  MicroPrintf("FAR-RANGE Detection System Ready!");
  MicroPrintf("Strategy: QVGA (320×240) → 4 ROIs → 96×96 model");
}

#if !CLI_ONLY_INFERENCE
// Declare external function to get ROI name
extern "C" const char* get_current_roi_name();

void loop() {
  static uint32_t frame_count = 0;
  static uint32_t last_detection_time = 0;
  
  // Process all 4 ROIs for one frame
  float max_person_score = 0.0f;
  float max_no_person_score = 1.0f;
  const char* best_roi = "";
  
  for (int roi_idx = 0; roi_idx < NUM_ROIS; roi_idx++) {
    // Get image for current ROI (GetImage automatically cycles through ROIs)
    if (kTfLiteOk != GetImage(kNumCols, kNumRows, kNumChannels, input->data.int8)) {
      MicroPrintf("Image capture failed for ROI %d", roi_idx);
      continue;
    }

    // Run inference on this ROI
    if (kTfLiteOk != interpreter->Invoke()) {
      MicroPrintf("Invoke failed for ROI %d", roi_idx);
      continue;
    }

    TfLiteTensor* output = interpreter->output(0);
    int8_t person_score = output->data.uint8[kPersonIndex];
    int8_t no_person_score = output->data.uint8[kNotAPersonIndex];

    float person_score_f = (person_score - output->params.zero_point) * output->params.scale;
    float no_person_score_f = (no_person_score - output->params.zero_point) * output->params.scale;

    // Track best detection across all ROIs
    if (person_score_f > max_person_score) {
      max_person_score = person_score_f;
      max_no_person_score = no_person_score_f;
      best_roi = get_current_roi_name();
    }
  }
  
  // Report best detection from all ROIs (once per second)
  uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
  if (current_time - last_detection_time >= 1000) {
    frame_count++;
    
    // Only print if person detected with confidence > 60%
    int person_pct = (int)(max_person_score * 100);
    if (person_pct >= 60) {
      MicroPrintf("[Frame %lu] PERSON DETECTED at %s: %d%%",
                  frame_count, best_roi, person_pct);
    } else {
      MicroPrintf("[Frame %lu] No person detected (max: %d%%)",
                  frame_count, person_pct);
    }
    
    RespondToDetection(max_person_score, max_no_person_score);
    last_detection_time = current_time;
  }
  
  vTaskDelay(10 / portTICK_PERIOD_MS);  // Small delay between ROI processing
}
#endif

void run_inference(void *ptr) {
  for (int i = 0; i < kNumCols * kNumRows; i++) {
    input->data.int8[i] = ((uint8_t *) ptr)[i] ^ 0x80;
  }

  if (kTfLiteOk != interpreter->Invoke()) {
    MicroPrintf("Invoke failed.");
  }

  TfLiteTensor* output = interpreter->output(0);
  int8_t person_score = output->data.uint8[kPersonIndex];
  int8_t no_person_score = output->data.uint8[kNotAPersonIndex];

  float person_score_f = (person_score - output->params.zero_point) * output->params.scale;
  float no_person_score_f = (no_person_score - output->params.zero_point) * output->params.scale;
  
  RespondToDetection(person_score_f, no_person_score_f);
}