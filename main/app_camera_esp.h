#ifndef TENSORFLOW_LITE_MICRO_EXAMPLES_PERSON_DETECTION_ESP_APP_CAMERA_ESP_H_
#define TENSORFLOW_LITE_MICRO_EXAMPLES_PERSON_DETECTION_ESP_APP_CAMERA_ESP_H_

#include "sensor.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_main.h"

/**
 * Camera Pixel Format
 * PIXFORMAT_RGB565,    // 2BPP/RGB565
 * PIXFORMAT_YUV422,    // 2BPP/YUV422
 * PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
 * PIXFORMAT_JPEG,      // JPEG/COMPRESSED
 * PIXFORMAT_RGB888,    // 3BPP/RGB888
 */
#if defined DISPLAY_SUPPORT
#define CAMERA_PIXEL_FORMAT PIXFORMAT_RGB565
#else
#define CAMERA_PIXEL_FORMAT PIXFORMAT_GRAYSCALE
#endif

/**
 * Camera Frame Size
 * FRAMESIZE_96X96,    // 96x96
 * FRAMESIZE_QQVGA,    // 160x120
 * FRAMESIZE_QVGA,     // 320x240
 * FRAMESIZE_VGA,      // 640x480
 */
#define CAMERA_FRAME_SIZE FRAMESIZE_96X96

/**
 * YOUR CUSTOM ESP32-S3 CAMERA PIN CONFIGURATION
 * Based on your hardware with OV2640 sensor
 */
#define CAMERA_MODULE_NAME "ESP32-S3-CUSTOM-OV2640"

// Power and reset pins
#define CAMERA_PIN_PWDN     -1      // Not used
#define CAMERA_PIN_RESET    47      // Reset pin

// Clock pins
#define CAMERA_PIN_XCLK     45      // Master clock
#define CAMERA_PIN_PCLK     15      // Pixel clock

// I2C pins for camera control (SCCB protocol)
#define CAMERA_PIN_SIOD     38      // I2C SDA (Data)
#define CAMERA_PIN_SIOC     39      // I2C SCL (Clock)

// Sync pins
#define CAMERA_PIN_VSYNC    40      // Vertical sync
#define CAMERA_PIN_HREF     48      // Horizontal reference

// Data pins (8-bit parallel interface)
#define CAMERA_PIN_D0       17      // Y2
#define CAMERA_PIN_D1       8       // Y3
#define CAMERA_PIN_D2       3       // Y4
#define CAMERA_PIN_D3       18      // Y5
#define CAMERA_PIN_D4       16      // Y6
#define CAMERA_PIN_D5       1       // Y7
#define CAMERA_PIN_D6       2       // Y8
#define CAMERA_PIN_D7       41      // Y9

// Camera clock frequency (10MHz works well with OV2640)
#define XCLK_FREQ_HZ 10000000

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize camera with custom pin configuration
 * Returns: 0 on success, -1 on failure
 */
int app_camera_init(void);

#ifdef __cplusplus
}
#endif

#endif  // TENSORFLOW_LITE_MICRO_EXAMPLES_PERSON_DETECTION_ESP_APP_CAMERA_ESP_H_