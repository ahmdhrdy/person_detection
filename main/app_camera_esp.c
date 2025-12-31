#include "app_camera_esp.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

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
#if ESP_CAMERA_SUPPORTED
  
  ESP_LOGI(TAG, "Initializing camera for FAR-RANGE person detection...");
  ESP_LOGI(TAG, "Strategy: QVGA (320x240) → Multiple 96x96 ROIs → Better far detection");
  
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

    // CRITICAL: Use QVGA for ROI extraction
    .xclk_freq_hz = 10000000,           // 10MHz clock
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    // ROI STRATEGY: QVGA 320×240 grayscale
    .pixel_format = PIXFORMAT_GRAYSCALE,  // Grayscale (model expects 1 channel)
    .frame_size = FRAMESIZE_QVGA,         // 320×240 (captures far people!)
    .jpeg_quality = 12,
    .fb_count = 2,                         // Double buffering
    .fb_location = CAMERA_FB_IN_PSRAM,    // Use PSRAM
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
  };

  ESP_LOGI(TAG, "Camera configuration:");
  ESP_LOGI(TAG, "  Resolution: QVGA (320×240) - Captures FAR people");
  ESP_LOGI(TAG, "  Format: GRAYSCALE");
  ESP_LOGI(TAG, "  ROI Strategy: 4 overlapping regions");
  ESP_LOGI(TAG, "  Model Input: 96×96 (unchanged)");

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed: 0x%x (%s)", err, esp_err_to_name(err));
    return -1;
  }

  ESP_LOGI(TAG, "Camera initialized successfully!");

  sensor_t *s = esp_camera_sensor_get();
  if (s == NULL) {
    ESP_LOGE(TAG, "Failed to get camera sensor");
    return -1;
  }

  // Optimize for person detection
  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);

  if (s->id.PID == OV2640_PID) {
    ESP_LOGI(TAG, "Detected OV2640 sensor - optimizing for far detection");
    s->set_brightness(s, 1);
    s->set_contrast(s, 1);
  }

  ESP_LOGI(TAG, "Camera ready for far-range person detection!");
  
  return 0;

#else
  ESP_LOGE(TAG, "Camera not supported!");
  return -1;
#endif
}