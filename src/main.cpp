#include <Arduino.h>
#include "esp_camera.h"
#include <WiFiRawComm.h>
#include <ESPNowCam.h>

// ----------------------------------------------------
// Seeed Studio XIAO ESP32S3 Sense OV2640 Pinout
// ----------------------------------------------------
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// Instantiate WiFi Raw Communication class and pass to ESPNowCam
WiFiRawComm wifiRaw;
ESPNowCam radio(&wifiRaw);

// Replace with your receiver's MAC address to improve quality 
// (For WiFi Raw broadcast, you can usually leave this empty, but setting the target improves packet delivery).
const uint8_t macRecv[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
    Serial.begin(115200);
    delay(1000);

    // 1. Camera Initialization
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG; // MJPEG Output
    
    // Set strictly to 320x240 for your LCD 
    config.frame_size = FRAMESIZE_QVGA;  
    
    // Compression quality: 10-15 is typically good (lower number = higher quality/size)
    config.jpeg_quality = 12; 
    config.fb_count = 2; // Requires BOARD_HAS_PSRAM defined in build flags
    config.grab_mode = CAMERA_GRAB_LATEST;

    // Init the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera Init Failed with error 0x%x\n", err);
        return;
    }
    Serial.println("Camera initialized successfully!");

    // 2. ESPNowCam Initialization over WiFi-Raw (802.11tx)
    radio.setTarget((uint8_t*)macRecv); // Sets receiver MAC address
    radio.setChannel(6);                // Channel to match with receiver (important!)
    radio.init(512);                    // 512 bytes is the standard recommended chunk size
    
    Serial.println("ESPNowCam WiFi-Raw mode started.");
}

void loop() {
    // Acquire a new MJPEG frame buffer
    camera_fb_t *fb = esp_camera_fb_get();
    
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }

    // Send raw MJPEG buffer over WiFi Raw
    radio.sendData(fb->buf, fb->len);
    
    // Return frame buffer so the DMA can reuse it
    esp_camera_fb_return(fb);
}
