#include "image_provider.h"
#include "app_camera_esp.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdlib>
#include <cstring>

static const char* TAG = "image_provider";

// ROI definitions - CENTER-FOCUSED for far detection
typedef struct {
  int x;
  int y;
  int w;
  int h;
  const char* name;
} ROI;

static const ROI rois[] = {
  {80, 0, 160, 160, "Top-Center"},
  {0, 40, 160, 160, "Left-Center"},
  {160, 40, 160, 160, "Right-Center"},
  {80, 80, 160, 160, "Bottom-Center"}
};
static const int NUM_ROIS = sizeof(rois) / sizeof(rois[0]);

static uint8_t roi_buffer[160 * 160];
static uint8_t resized_96[96 * 96];

static int current_roi_index = 0;

// Crop ROI from source frame
void crop_roi(const uint8_t* src, int src_w, int src_h,
              int x, int y, int w, int h, uint8_t* dst) {
  for (int row = 0; row < h; row++) {
    if ((y + row) < src_h) {
      int copy_width = (x + w <= src_w) ? w : (src_w - x);
      memcpy(dst + row * w, src + (y + row) * src_w + x, copy_width);
    }
  }
}

// Resize using nearest-neighbor
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

// Get ROI name
extern "C" const char* get_current_roi_name() {
  int prev_index = (current_roi_index - 1 + NUM_ROIS) % NUM_ROIS;
  return rois[prev_index].name;
}

TfLiteStatus GetImage(int image_width, int image_height, int channels, int8_t* image_data) {
#if ESP_CAMERA_SUPPORTED
  static camera_fb_t* fb = NULL;
  
  // On first ROI, capture new frame
  if (current_roi_index == 0) {
    if (fb != NULL) {
      esp_camera_fb_return(fb);
      fb = NULL;
    }
    
    // ULTRA-AGGRESSIVE RETRY for QVGA
    int retry_count = 0;
    const int MAX_RETRIES = 15;  // More retries
    
    while (retry_count < MAX_RETRIES) {
      // CRITICAL: Wait BEFORE capture
      vTaskDelay(pdMS_TO_TICKS(150));  // 150ms delay BEFORE each attempt!
      
      fb = esp_camera_fb_get();
      
      if (!fb) {
        ESP_LOGW(TAG, "Capture failed, retry %d/%d", retry_count + 1, MAX_RETRIES);
        retry_count++;
        continue;
      }
      
      // Validate frame size
      const size_t expected_size = 320 * 240;
      if (fb->len == expected_size) {
        // SUCCESS!
        break;
      }
      
      // Invalid size
      ESP_LOGW(TAG, "Invalid size: %zu (expected %zu), retry %d/%d", 
               fb->len, expected_size, retry_count + 1, MAX_RETRIES);
      esp_camera_fb_return(fb);
      fb = NULL;
      retry_count++;
      
      // CRITICAL: Long wait after bad frame
      vTaskDelay(pdMS_TO_TICKS(300));  // 300ms wait!
    }
    
    if (fb == NULL || fb->len != 320 * 240) {
      ESP_LOGE(TAG, "Failed to get valid frame after %d retries", MAX_RETRIES);
      return kTfLiteError;
    }
  }
  
  // Process current ROI
  const ROI* roi = &rois[current_roi_index];
  
  // Crop 160×160 ROI
  crop_roi((uint8_t*)fb->buf, 320, 240,
           roi->x, roi->y, roi->w, roi->h,
           roi_buffer);
  
  // Resize to 96×96
  resize_nearest_neighbor(roi_buffer, 160, 160,
                          resized_96, 96, 96);
  
  // Quantize to INT8
  for (int i = 0; i < 96 * 96; i++) {
    image_data[i] = (int8_t)(resized_96[i] - 128);
  }
  
  // Move to next ROI
  current_roi_index = (current_roi_index + 1) % NUM_ROIS;
  
  // If completed all ROIs, return frame
  if (current_roi_index == 0 && fb != NULL) {
    esp_camera_fb_return(fb);
    fb = NULL;
    
    // CRITICAL: Wait after returning frame
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms wait
  }
  
  return kTfLiteOk;
  
#else
  return kTfLiteError;
#endif
}

// Initialize camera
TfLiteStatus InitCamera() {
#if ESP_CAMERA_SUPPORTED
  ESP_LOGI(TAG, "Camera Initialized for ROI detection\n");
  ESP_LOGI(TAG, "ROI Strategy: Processing %d regions per frame", NUM_ROIS);
  
  int ret = app_camera_init();
  if (ret != 0) {
    return kTfLiteError;
  }
  return kTfLiteOk;
#else
  return kTfLiteError;
#endif
}