// ESP32S3-Seeed-Wifi-Raw-Cam\src\main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include "esp_camera.h"

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

uint8_t pyControllerMac[6];
uint8_t myMac[6];
volatile bool isConnected = false;
volatile bool captureRequested = false;
volatile bool is_streaming = false;

static uint16_t raw_seq = 0;

// ----------------------------------------------------
// ESP-NOW Receive Callback (Listens for Handshake / Controls)
// ----------------------------------------------------
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len >= 14 && strncmp((const char*)incomingData, "pyCAR_DISCOVER", 14) == 0) {
        if (!isConnected) {
            Serial.println("Received 'pyCAR_DISCOVER' via ESP-NOW!");
            memcpy(pyControllerMac, mac, 6);
            
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, pyControllerMac, 6);
            peerInfo.channel = 1; 
            peerInfo.encrypt = false;
            
            if (!esp_now_is_peer_exist(pyControllerMac)) {
                esp_now_add_peer(&peerInfo);
            }
            isConnected = true;
        }
        
        const char* ackMsg = "pyCAM_ACK";
        esp_now_send(pyControllerMac, (uint8_t *)ackMsg, strlen(ackMsg));
    } 
    else if (len >= 9 && strncmp((const char*)incomingData, "pyCAM_REQ", 9) == 0) {
        captureRequested = true;
    }
    else if (len >= 11 && strncmp((const char*)incomingData, "pyCAM_STR_1", 11) == 0) {
        is_streaming = true;
    }
    else if (len >= 11 && strncmp((const char*)incomingData, "pyCAM_STR_0", 11) == 0) {
        is_streaming = false;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // ---  1. Wi-Fi & ESP-NOW Init ---
    WiFi.mode(WIFI_STA);
    esp_read_mac(myMac, ESP_MAC_WIFI_STA);
    
    // Use native ESP-IDF API to set the Wi-Fi channel securely
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
    Serial.println("Waiting for PyController to broadcast 'pyCAR_DISCOVER'...");

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
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_QVGA; 
    config.jpeg_quality = 12; 
    config.fb_count     = 2;  
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera Init Failed");
        return;
    }
    Serial.println("Camera initialized!");
}

void loop() {
    if ((captureRequested || is_streaming) && isConnected) {
        captureRequested = false;
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            // Fragment JPEG payload sequentially across massive RAW Wi-Fi Frames
            int max_payload = 1000; 
            uint16_t total_chunks = (fb->len + max_payload - 1) / max_payload;
            
            for (uint16_t i = 0; i < total_chunks; i++) {
                uint8_t packet[1100];
                
                // --- 802.11 MAC Header (24 bytes) ---
                packet[0] = 0x08; // Frame Control: Data frame (Subtype 0)
                packet[1] = 0x00; // Flags
                packet[2] = 0x00; // Duration
                packet[3] = 0x00;
                memcpy(&packet[4], pyControllerMac, 6);  // Addr1: Dest
                memcpy(&packet[10], myMac, 6);           // Addr2: Src
                memcpy(&packet[16], pyControllerMac, 6); // Addr3: BSSID
                packet[22] = (raw_seq & 0x0F) << 4;      // Seq LSB
                packet[23] = (raw_seq >> 4) & 0xFF;      // Seq MSB
                raw_seq++;

                // --- Custom Header (9 bytes) ---
                packet[24] = 'C';
                packet[25] = 'A';
                packet[26] = 'M';
                memcpy(&packet[27], &i, 2);
                memcpy(&packet[29], &total_chunks, 2);
                
                int offset = i * max_payload;
                uint16_t len = fb->len - offset;
                if (len > max_payload) len = max_payload;
                
                memcpy(&packet[31], &len, 2);

                // --- JPEG Data ---
                memcpy(&packet[33], fb->buf + offset, len);
                
                // Inject the packet directly to the baseband
                esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, packet, 33 + len, false);
                if (err != ESP_OK) {
                    delay(1); // Give buffers time to empty if we hit ESP_ERR_NO_MEM
                    esp_wifi_80211_tx(WIFI_IF_STA, packet, 33 + len, false);
                } else {
                    delayMicroseconds(300); // Small 300us inter-packet gap to avoid overflowing receiver
                }
            }
            esp_camera_fb_return(fb);
        }
    }
    delay(10);
}
