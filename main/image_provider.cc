/* Copyright 2019 The TensorFlow Authors. All Rights Reserved. */

#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if (CONFIG_TFLITE_USE_BSP)
#include "bsp/esp-bsp.h"
#endif

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "app_camera_esp.h"
#include "esp_camera.h"
#include "model_settings.h"
#include "image_provider.h"
#include "esp_main.h"

// Forward declaration for C linkage
#ifdef __cplusplus
extern "C" {
#endif

const char* get_current_roi_name();

#ifdef __cplusplus
}
#endif

static const char* TAG = "image_provider";
static uint16_t* display_buf;

// ROI STRATEGY: 4 overlapping regions from 320×240 frame
typedef struct {
  int x, y, w, h;
  const char* name;
} ROI;

// Define 4 ROIs covering the frame strategically
static const ROI rois[] = {
  {0, 0, 160, 160, "Top-Left"},       // Covers top-left area
  {160, 0, 160, 160, "Top-Right"},    // Covers top-right area
  {0, 80, 160, 160, "Bottom-Left"},   // Covers bottom-left area
  {160, 80, 160, 160, "Bottom-Right"} // Covers bottom-right area
};
static const int NUM_ROIS = sizeof(rois) / sizeof(rois[0]);

// Buffers for ROI processing
static uint8_t roi_buffer[160 * 160];  // Cropped ROI
static uint8_t resized_96[96 * 96];    // Resized to 96×96

// FUNCTION: Crop ROI from source frame
void crop_roi(const uint8_t* src, int src_w, int src_h,
              int x, int y, int w, int h, uint8_t* dst) {
  for (int row = 0; row < h; row++) {
    if ((y + row) < src_h) {
      int copy_width = (x + w <= src_w) ? w : (src_w - x);
      memcpy(dst + row * w, src + (y + row) * src_w + x, copy_width);
    }
  }
}

// FUNCTION: Resize using nearest-neighbor (FAST on ESP32)
void resize_nearest_neighbor(const uint8_t* src, int sw, int sh,
                             uint8_t* dst, int dw, int dh) {
  for (int y = 0; y < dh; y++) {
    int sy = (y * sh) / dh;
    for (int x = 0; x < dw; x++) {
      int sx = (x * sw) / dw;
      dst[y * dw + x] = src[sy * sw + sx];
    }
  }
}

// Get the camera module ready
TfLiteStatus InitCamera() {
#if CLI_ONLY_INFERENCE
  ESP_LOGI(TAG, "CLI_ONLY_INFERENCE enabled, skipping camera init");
  return kTfLiteOk;
#endif

#if DISPLAY_SUPPORT
  if (display_buf == NULL) {
    display_buf = (uint16_t *) heap_caps_malloc(96 * 2 * 96 * 2 * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  if (display_buf == NULL) {
    ESP_LOGE(TAG, "Couldn't allocate display buffer");
    return kTfLiteError;
  }
#endif

#if ESP_CAMERA_SUPPORTED
  int ret = app_camera_init();
  if (ret != 0) {
    MicroPrintf("Camera init failed\n");
    return kTfLiteError;
  }
  MicroPrintf("Camera Initialized for ROI detection\n");
  ESP_LOGI(TAG, "ROI Strategy: Processing %d regions per frame", NUM_ROIS);
#else
  ESP_LOGE(TAG, "Camera not supported for this device");
#endif
  return kTfLiteOk;
}

void *image_provider_get_display_buf() {
  return (void *) display_buf;
}

// GLOBAL: Store current ROI index for multi-ROI detection
static int current_roi_index = 0;

// Get an image from the camera module (ONE ROI AT A TIME)
TfLiteStatus GetImage(int image_width, int image_height, int channels, int8_t* image_data) {
#if ESP_CAMERA_SUPPORTED
  static camera_fb_t* fb = NULL;
  
  // On first ROI (index 0), capture new frame
  if (current_roi_index == 0) {
    if (fb != NULL) {
      esp_camera_fb_return(fb);  // Return previous frame
    }
    fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGE(TAG, "Camera capture failed");
      return kTfLiteError;
    }
  }
  
  // Process current ROI
  const ROI* roi = &rois[current_roi_index];
  
  // Step 1: Crop ROI from 320×240 frame
  crop_roi((uint8_t*)fb->buf, 320, 240,
           roi->x, roi->y, roi->w, roi->h,
           roi_buffer);
  
  // Step 2: Resize 160×160 ROI → 96×96
  resize_nearest_neighbor(roi_buffer, 160, 160,
                          resized_96, 96, 96);
  
  // Step 3: Quantize to INT8 for model
  for (int i = 0; i < 96 * 96; i++) {
    image_data[i] = resized_96[i] ^ 0x80;  // Convert to signed INT8
  }
  
  // Move to next ROI (wrap around)
  current_roi_index = (current_roi_index + 1) % NUM_ROIS;
  
  // If we completed all ROIs, return the frame
  if (current_roi_index == 0 && fb != NULL) {
    esp_camera_fb_return(fb);
    fb = NULL;
  }
  
  return kTfLiteOk;
#else
  return kTfLiteError;
#endif
}

// HELPER: Get current ROI name for logging (extern C for linking)
extern "C" const char* get_current_roi_name() {
  int prev_index = (current_roi_index - 1 + NUM_ROIS) % NUM_ROIS;
  return rois[prev_index].name;
}