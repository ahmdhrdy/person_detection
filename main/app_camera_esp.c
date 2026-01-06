#include "app_camera_esp.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_camera";

// Camera pin definitions
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    47
#define XCLK_GPIO_NUM     45
#define SIOD_GPIO_NUM     38
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       41
#define Y8_GPIO_NUM       2
#define Y7_GPIO_NUM       1
#define Y6_GPIO_NUM       16
#define Y5_GPIO_NUM       18
#define Y4_GPIO_NUM       3
#define Y3_GPIO_NUM       8
#define Y2_GPIO_NUM       17
#define VSYNC_GPIO_NUM    40
#define HREF_GPIO_NUM     48
#define PCLK_GPIO_NUM     15

int app_camera_init() {
  ESP_LOGI(TAG, "═══════════════════════════════════════════════");
  ESP_LOGI(TAG, "Initializing camera for FAR-RANGE detection");
  ESP_LOGI(TAG, "Strategy: QVGA (320×240) → 4 ROIs → 96×96 model");
  ESP_LOGI(TAG, "═══════════════════════════════════════════════");

  camera_config_t config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    // CRITICAL: Slower clock for QVGA stability
    .xclk_freq_hz = 10000000,           // 10MHz (not 20MHz!)
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAMESIZE_QVGA,       // 320×240 for far detection

    .jpeg_quality = 12,
    .fb_count = 2,                      // Double buffering
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
  };

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
    return -1;
  }

  ESP_LOGI(TAG, "Camera initialized successfully!");

  sensor_t *s = esp_camera_sensor_get();
  if (s == NULL) {
    ESP_LOGE(TAG, "Failed to get camera sensor");
    return -1;
  }

  // Optimize settings
  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);

  if (s->id.PID == OV2640_PID) {
    ESP_LOGI(TAG, "Detected OV2640 - optimizing for QVGA");
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
  }

  // CRITICAL: Long stabilization for QVGA
  ESP_LOGI(TAG, "Waiting for camera to stabilize (2 seconds)...");
  vTaskDelay(pdMS_TO_TICKS(2000));  // 2 seconds!

  // Discard first 5 frames
  ESP_LOGI(TAG, "Discarding warm-up frames...");
  for (int i = 0; i < 5; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      ESP_LOGI(TAG, "  Frame %d: size=%zu", i + 1, fb->len);
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  ESP_LOGI(TAG, "═══════════════════════════════════════════════");
  ESP_LOGI(TAG, "Camera ready for far-range detection!");
  ESP_LOGI(TAG, "═══════════════════════════════════════════════");

  return 0;
}