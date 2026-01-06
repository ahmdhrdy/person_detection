// #include "main_functions.h"

// #include "detection_responder.h"
// #include "image_provider.h"
// #include "model_settings.h"
// #include "person_detect_model_data.h"
// #include "tensorflow/lite/micro/micro_interpreter.h"
// #include "tensorflow/lite/micro/micro_log.h"
// #include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
// #include "tensorflow/lite/schema/schema_generated.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include <esp_heap_caps.h>
// #include <esp_timer.h>
// #include <esp_log.h>
// #include "esp_main.h"

// namespace {
// const tflite::Model* model = nullptr;
// tflite::MicroInterpreter* interpreter = nullptr;
// TfLiteTensor* input = nullptr;

// #if CONFIG_NN_OPTIMIZED
// constexpr int scratchBufSize = 60 * 1024;
// #else
// constexpr int scratchBufSize = 0;
// #endif

// constexpr int kTensorArenaSize = 100 * 1024 + scratchBufSize;
// static uint8_t *tensor_arena;

// constexpr int NUM_ROIS = 4;

// // ═══════════════════════════════════════════════════════
// // CONFIDENCE AVERAGING (FIX FOR FALSE POSITIVES!)
// // ═══════════════════════════════════════════════════════
// constexpr int CONFIDENCE_HISTORY_SIZE = 5;
// static float confidence_history[CONFIDENCE_HISTORY_SIZE] = {0};
// static int confidence_index = 0;
// }  // namespace

// #define LED_GPIO GPIO_NUM_12 


// void setup() {

//   model = tflite::GetModel(g_person_detect_model_data);
//   if (model->version() != TFLITE_SCHEMA_VERSION) {
//     MicroPrintf("Model schema version mismatch!");
//     return;
//   }

//   if (tensor_arena == NULL) {
//     tensor_arena = (uint8_t *) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
//   }
//   if (tensor_arena == NULL) {
//     MicroPrintf("Memory allocation failed!");
//     return;
//   }

//   static tflite::MicroMutableOpResolver<5> micro_op_resolver;
//   micro_op_resolver.AddAveragePool2D();
//   micro_op_resolver.AddConv2D();
//   micro_op_resolver.AddDepthwiseConv2D();
//   micro_op_resolver.AddReshape();
//   micro_op_resolver.AddSoftmax();

//   static tflite::MicroInterpreter static_interpreter(
//       model, micro_op_resolver, tensor_arena, kTensorArenaSize);
//   interpreter = &static_interpreter;

//   TfLiteStatus allocate_status = interpreter->AllocateTensors();
//   if (allocate_status != kTfLiteOk) {
//     MicroPrintf("AllocateTensors() failed");
//     return;
//   }

//   input = interpreter->input(0);

// #if !CLI_ONLY_INFERENCE
//   TfLiteStatus init_status = InitCamera();
//   if (init_status != kTfLiteOk) {
//     MicroPrintf("InitCamera failed\n");
//     return;
//   }

// #if DISPLAY_SUPPORT
//   create_gui();
// #endif
// #endif

//   MicroPrintf("═══════════════════════════════════════════════════════");
//   MicroPrintf("FAR-RANGE Detection System Ready!");
//   MicroPrintf("Strategy: QVGA (320×240) → 4 ROIs → 96×96 model");
//   MicroPrintf("False Positive Fix: 5-frame confidence averaging");
//   MicroPrintf("═══════════════════════════════════════════════════════");
// }

// #if !CLI_ONLY_INFERENCE
// extern "C" const char* get_current_roi_name();

// void loop() {
//   static uint32_t frame_count = 0;
//   static uint32_t last_detection_time = 0;
//   static uint32_t last_frame_time = 0;
  
//   // Process all 4 ROIs for one frame
//   float max_person_score = 0.0f;
//   const char* best_roi = "";
  
//   for (int roi_idx = 0; roi_idx < NUM_ROIS; roi_idx++) {
//     if (kTfLiteOk != GetImage(kNumCols, kNumRows, kNumChannels, input->data.int8)) {
//       MicroPrintf("⚠️ Image capture failed for ROI %d", roi_idx);
//       vTaskDelay(200 / portTICK_PERIOD_MS);  // Wait on error
//       continue;
//     }

//     if (kTfLiteOk != interpreter->Invoke()) {
//       MicroPrintf("⚠️ Invoke failed for ROI %d", roi_idx);
//       continue;
//     }

//     TfLiteTensor* output = interpreter->output(0);
//     int8_t person_score = output->data.int8[kPersonIndex];
//     int8_t no_person_score = output->data.int8[kNotAPersonIndex];

//     float person_score_f = (person_score - output->params.zero_point) * output->params.scale;

//     if (person_score_f > max_person_score) {
//       max_person_score = person_score_f;
//       best_roi = get_current_roi_name();
//     }
//   }
  
//   // ═══════════════════════════════════════════════════════
//   // CONFIDENCE AVERAGING (KEY FIX FOR FALSE POSITIVES!)
//   // ═══════════════════════════════════════════════════════
  
//   confidence_history[confidence_index] = max_person_score;
//   confidence_index = (confidence_index + 1) % CONFIDENCE_HISTORY_SIZE;
  
//   float avg_confidence = 0.0f;
//   for (int i = 0; i < CONFIDENCE_HISTORY_SIZE; i++) {
//     avg_confidence += confidence_history[i];
//   }
//   avg_confidence /= CONFIDENCE_HISTORY_SIZE;
  
//   // ═══════════════════════════════════════════════════════
//   // DETECTION DECISION
//   // ═══════════════════════════════════════════════════════
  
//   uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
//   if (current_time - last_detection_time >= 1000) {
//     frame_count++;
    
//     int instant_pct = (int)(max_person_score * 100);
//     int avg_pct = (int)(avg_confidence * 100);
    
//     // Calculate FPS
//     uint32_t frame_time_ms = current_time - last_frame_time;
//     float fps = (frame_time_ms > 0) ? (1000.0f / frame_time_ms) : 0;
//     last_frame_time = current_time;
    
//     // ROBUST DETECTION: Both high average AND reasonable instant
//     bool person_detected = (avg_pct >= 75) && (instant_pct >= 70);
    
//     if (person_detected) {
//       MicroPrintf("═══════════════════════════════════════════════════════");
//       MicroPrintf("[Frame %lu] ✅ PERSON DETECTED at %s", frame_count, best_roi);
//       MicroPrintf("  Instant: %d%% | Average: %d%% | FPS: %.1f", instant_pct, avg_pct, fps);
//       MicroPrintf("═══════════════════════════════════════════════════════");
//     } else {
//       if (instant_pct > 60) {
//         // High instant but low average = FILTERED OUT
//         MicroPrintf("[Frame %lu] ⚠️ REJECTED (High inst, low avg)", frame_count);
//         MicroPrintf("  Instant: %d%% | Avg: %d%% | FPS: %.1f → FILTERED", 
//                     instant_pct, avg_pct, fps);
//       } else {
//         // Clear no-person
//         MicroPrintf("[Frame %lu] ❌ No person | Inst: %d%% | Avg: %d%% | FPS: %.1f",
//                     frame_count, instant_pct, avg_pct, fps);
//       }
//     }
    
//     RespondToDetection(avg_confidence, 1.0f - avg_confidence);
//     last_detection_time = current_time;
//   }
  
//   // Moderate delay
//   vTaskDelay(200 / portTICK_PERIOD_MS);
// }
// #endif

// void run_inference(void *ptr) {
//   for (int i = 0; i < kNumCols * kNumRows; i++) {
//     input->data.int8[i] = (int8_t)(((uint8_t *) ptr)[i] - 128);
//   }

//   if (kTfLiteOk != interpreter->Invoke()) {
//     MicroPrintf("Invoke failed.");
//   }

//   TfLiteTensor* output = interpreter->output(0);
//   int8_t person_score = output->data.int8[kPersonIndex];
//   int8_t no_person_score = output->data.int8[kNotAPersonIndex];

//   float person_score_f = (person_score - output->params.zero_point) * output->params.scale;
//   float no_person_score_f = (no_person_score - output->params.zero_point) * output->params.scale;
  
//   RespondToDetection(person_score_f, no_person_score_f);
// }



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

// ═══════════════════════════════════════════════════════
// LED GPIO CONFIGURATION
// ═══════════════════════════════════════════════════════
#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_12  // Change to your LED pin

static void init_led() {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << LED_PIN);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);
  
  // Start with LED OFF
  gpio_set_level(LED_PIN, 0);
  
  ESP_LOGI("LED", "LED initialized on GPIO %d", LED_PIN);
}

static void set_led(bool on) {
  gpio_set_level(LED_PIN, on ? 1 : 0);
}

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

constexpr int NUM_ROIS = 4;

// ═══════════════════════════════════════════════════════
// CONFIDENCE AVERAGING (FIX FOR FALSE POSITIVES!)
// ═══════════════════════════════════════════════════════
constexpr int CONFIDENCE_HISTORY_SIZE = 5;
static float confidence_history[CONFIDENCE_HISTORY_SIZE] = {0};
static int confidence_index = 0;
}  // namespace

void setup() {
  model = tflite::GetModel(g_person_detect_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model schema version mismatch!");
    return;
  }

  if (tensor_arena == NULL) {
    tensor_arena = (uint8_t *) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (tensor_arena == NULL) {
    MicroPrintf("Memory allocation failed!");
    return;
  }

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

  // Initialize LED
  init_led();

  MicroPrintf("═══════════════════════════════════════════════════════");
  MicroPrintf("FAR-RANGE Detection System Ready!");
  MicroPrintf("Strategy: QVGA (320×240) → 4 ROIs → 96×96 model");
  MicroPrintf("False Positive Fix: 5-frame confidence averaging");
  MicroPrintf("LED Output: GPIO %d (ON=Person, OFF=No Person)", LED_PIN);
  MicroPrintf("═══════════════════════════════════════════════════════");
}

#if !CLI_ONLY_INFERENCE
extern "C" const char* get_current_roi_name();

void loop() {
  static uint32_t frame_count = 0;
  static uint32_t last_detection_time = 0;
  static uint32_t last_frame_time = 0;
  
  // Process all 4 ROIs for one frame
  float max_person_score = 0.0f;
  const char* best_roi = "";
  
  for (int roi_idx = 0; roi_idx < NUM_ROIS; roi_idx++) {
    if (kTfLiteOk != GetImage(kNumCols, kNumRows, kNumChannels, input->data.int8)) {
      MicroPrintf("⚠️ Image capture failed for ROI %d", roi_idx);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      continue;
    }

    if (kTfLiteOk != interpreter->Invoke()) {
      MicroPrintf("⚠️ Invoke failed for ROI %d", roi_idx);
      continue;
    }

    TfLiteTensor* output = interpreter->output(0);
    int8_t person_score = output->data.int8[kPersonIndex];

    float person_score_f = (person_score - output->params.zero_point) * output->params.scale;

    if (person_score_f > max_person_score) {
      max_person_score = person_score_f;
      best_roi = get_current_roi_name();
    }
  }
  
  // ═══════════════════════════════════════════════════════
  // CONFIDENCE AVERAGING (KEY FIX FOR FALSE POSITIVES!)
  // ═══════════════════════════════════════════════════════
  
  confidence_history[confidence_index] = max_person_score;
  confidence_index = (confidence_index + 1) % CONFIDENCE_HISTORY_SIZE;
  
  float avg_confidence = 0.0f;
  for (int i = 0; i < CONFIDENCE_HISTORY_SIZE; i++) {
    avg_confidence += confidence_history[i];
  }
  avg_confidence /= CONFIDENCE_HISTORY_SIZE;
  
  // ═══════════════════════════════════════════════════════
  // DETECTION DECISION + LED CONTROL
  // ═══════════════════════════════════════════════════════
  
  uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
  if (current_time - last_detection_time >= 1000) {
    frame_count++;
    
    int instant_pct = (int)(max_person_score * 100);
    int avg_pct = (int)(avg_confidence * 100);
    
    // Calculate FPS
    uint32_t frame_time_ms = current_time - last_frame_time;
    float fps = (frame_time_ms > 0) ? (1000.0f / frame_time_ms) : 0;
    last_frame_time = current_time;
    
    // ROBUST DETECTION: Both high average AND reasonable instant
    bool person_detected = (avg_pct >= 75) && (instant_pct >= 70);
    
    // ═══════════════════════════════════════════════════════
    // LED CONTROL: ON = Person, OFF = No Person
    // ═══════════════════════════════════════════════════════
    set_led(person_detected);
    
    if (person_detected) {
      MicroPrintf("═══════════════════════════════════════════════════════");
      MicroPrintf("[Frame %lu] ✅ PERSON DETECTED at %s | 🔴 LED ON", frame_count, best_roi);
      MicroPrintf("  Instant: %d%% | Average: %d%% | FPS: %.1f", instant_pct, avg_pct, fps);
      MicroPrintf("═══════════════════════════════════════════════════════");
    } else {
      if (instant_pct > 60) {
        // High instant but low average = FILTERED OUT
        MicroPrintf("[Frame %lu] ⚠️ REJECTED (High inst, low avg) | LED OFF", frame_count);
        MicroPrintf("  Instant: %d%% | Avg: %d%% | FPS: %.1f → FILTERED", 
                    instant_pct, avg_pct, fps);
      } else {
        // Clear no-person
        MicroPrintf("[Frame %lu] ❌ No person | Inst: %d%% | Avg: %d%% | LED OFF | FPS: %.1f",
                    frame_count, instant_pct, avg_pct, fps);
      }
    }
    
    RespondToDetection(avg_confidence, 1.0f - avg_confidence);
    last_detection_time = current_time;
  }
  
  // Moderate delay
  vTaskDelay(200 / portTICK_PERIOD_MS);
}
#endif

void run_inference(void *ptr) {
  for (int i = 0; i < kNumCols * kNumRows; i++) {
    input->data.int8[i] = (int8_t)(((uint8_t *) ptr)[i] - 128);
  }

  if (kTfLiteOk != interpreter->Invoke()) {
    MicroPrintf("Invoke failed.");
  }

  TfLiteTensor* output = interpreter->output(0);
  int8_t person_score = output->data.int8[kPersonIndex];
  int8_t no_person_score = output->data.int8[kNotAPersonIndex];

  float person_score_f = (person_score - output->params.zero_point) * output->params.scale;
  float no_person_score_f = (no_person_score - output->params.zero_point) * output->params.scale;
  
  RespondToDetection(person_score_f, no_person_score_f);
}