#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel
#include "esp_camera.h"

// Include ESPNowCam and the WiFi Raw Comm Wrapper
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

// Instantiate WiFi Raw Communication for the camera
WiFiRawComm wifiRaw;
ESPNowCam radio(&wifiRaw);

// Connection state variables
uint8_t pyControllerMac[6];
volatile bool isConnected = false;

// ----------------------------------------------------
// ESP-NOW Receive Callback (Listens for Handshake / Controls)
// ----------------------------------------------------
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    // 1. Handshake Phase: Listen for MicroPython broadcast
    if (!isConnected && len >= 14 && strncmp((const char*)incomingData, "pyCAR_DISCOVER", 14) == 0) {
        Serial.println("Received 'pyCAR_DISCOVER' via ESP-NOW!");
        memcpy(pyControllerMac, mac, 6);
        
        // Register the PyController to send replies
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, pyControllerMac, 6);
        peerInfo.channel = 1; // Channel must match python script
        peerInfo.encrypt = false;
        
        if (!esp_now_is_peer_exist(pyControllerMac)) {
            esp_now_add_peer(&peerInfo);
        }

        // Send ACK back via standard ESP-NOW
        const char* ackMsg = "pyCAR_ACK";
        esp_now_send(pyControllerMac, (uint8_t *)ackMsg, strlen(ackMsg));
        
        Serial.println("Sent 'pyCAR_ACK' via ESP-NOW. Proceeding to boot camera!");
        isConnected = true;
    } 
    // 2. Control Phase: Listen for standard ESP-NOW joystick data
    else if (isConnected && len == 6 && incomingData[0] == 67) {
        // incomingData contains your struct.pack('<BBBBBB', 67, key[1], ...)
        uint8_t joyL_x = incomingData[1];
        uint8_t joyL_y = incomingData[2];
        uint8_t joyR_x = incomingData[3];
        uint8_t joyR_y = incomingData[4];
        uint8_t buttons = incomingData[5];
        
        // E.g., apply motor speeds here
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // --- 1. Wi-Fi & Standard ESP-NOW Init ---
    WiFi.mode(WIFI_STA);
    
    // Set Wi-Fi Channel using native ESP-IDF function (Fix for compile error)
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); 

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register callback for auto-connection
    esp_now_register_recv_cb(onDataRecv);

    Serial.println("Waiting for PyController to broadcast 'pyCAR_DISCOVER'...");
    
    // Block execution until we perform the ESP-NOW handshake
    while (!isConnected) {
        delay(100);
    }

    // --- 2. Camera Initialization ---
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
    config.pixel_format = PIXFORMAT_JPEG; // Required for MJPEG
    config.frame_size   = FRAMESIZE_QVGA; // Exactly 320x240 to fit your LCD
    config.jpeg_quality = 12; // Lower = higher quality
    config.fb_count     = 2;  // Requires PSRAM flag in PlatformIO
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera Init Failed");
        return;
    }
    Serial.println("Camera initialized!");

    // --- 3. Start WiFi Raw Comm for Video Streaming ---
    // WiFi Raw supports much larger packets than ESP-NOW. 512 or 1000 bytes works great.
    radio.setTarget(pyControllerMac); // Address the Raw frames to your PyController MAC
    radio.setChannel(1);              // Keep it on the same Wi-Fi channel
    radio.init(512);                  // 512 Byte payload chunks
    
    Serial.println("Video Streaming via Raw 802.11tx Started.");
}

void loop() {
    if (isConnected) {
        // Capture frame from OV2640
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            // Push high-bandwidth MJPEG data via RAW WiFi frames (Not ESP-NOW)
            radio.sendData(fb->buf, fb->len);
            
            // Return buffer to the DMA
            esp_camera_fb_return(fb);
        }
    }
}
