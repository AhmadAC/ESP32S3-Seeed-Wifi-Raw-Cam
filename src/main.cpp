// ESP32S3-Seeed-Wifi-Raw-Cam/src/main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <esp_idf_version.h>
#include "esp_camera.h"
#include <WebServer.h>

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
uint8_t cam_mac[6];
volatile bool isConnected = false;
volatile bool captureRequested = false;
volatile bool is_streaming = false;

// State management for ESP-NOW and Access Point modes
enum SystemMode {
    MODE_WAITING,
    MODE_ESPNOW,
    MODE_AP
};

volatile SystemMode currentMode = MODE_WAITING;
unsigned long startWaitTime = 0;
const unsigned long ESPNOW_TIMEOUT_MS = 7000; // 7 seconds timeout to look for ESP-NOW peer
bool apStarted = false;

WebServer server(80);

// Boundary definitions for MJPEG Streaming
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// HTML, CSS, and JS interface served to connected phones/devices
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32S3 Camera Stream</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #121212;
            color: #ffffff;
            text-align: center;
            margin: 0;
            padding: 20px;
        }
        h1 {
            color: #00adb5;
            margin-bottom: 20px;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
            background-color: #1e1e1e;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 4px 10px rgba(0,0,0,0.5);
        }
        .stream-container {
            width: 100%;
            border-radius: 8px;
            overflow: hidden;
            background-color: #000;
            margin-bottom: 20px;
            position: relative;
        }
        img {
            display: block;
            width: 100%;
            height: auto;
        }
        .btn {
            background-color: #00adb5;
            color: #ffffff;
            border: none;
            padding: 12px 24px;
            font-size: 16px;
            font-weight: bold;
            border-radius: 5px;
            cursor: pointer;
            transition: background-color 0.3s ease;
            box-shadow: 0 2px 5px rgba(0,0,0,0.3);
        }
        .btn:hover {
            background-color: #007a80;
        }
        .btn:disabled {
            background-color: #555555;
            cursor: not-allowed;
        }
        .status-text {
            margin-top: 10px;
            color: #aaaaaa;
            font-size: 14px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32S3 Camera Feed</h1>
        <div class="stream-container">
            <img src="/stream" alt="Live Stream">
        </div>
        <button id="snapBtn" class="btn" onclick="takeSnapshot()">Save Snapshot</button>
        <div id="status" class="status-text">Streaming live...</div>
    </div>

    <script>
        function takeSnapshot() {
            const btn = document.getElementById('snapBtn');
            const status = document.getElementById('status');
            btn.disabled = true;
            status.innerText = "Capturing high-resolution image...";

            fetch('/capture')
                .then(response => {
                    if (!response.ok) throw new Error('Capture failed');
                    return response.blob();
                })
                .then(blob => {
                    const url = window.URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.style.display = 'none';
                    a.href = url;
                    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
                    a.download = `snapshot_${timestamp}.jpg`;
                    document.body.appendChild(a);
                    a.click();
                    window.URL.revokeObjectURL(url);
                    document.body.removeChild(a);
                    status.innerText = "Snapshot saved!";
                    btn.disabled = false;
                    setTimeout(() => {
                        status.innerText = "Streaming live...";
                    }, 3000);
                })
                .catch(err => {
                    console.error(err);
                    status.innerText = "Error taking snapshot.";
                    btn.disabled = false;
                });
        }
    </script>
</body>
</html>
)rawliteral";

// Web server request handlers
void handleRoot() {
    server.send(200, "text/html", INDEX_HTML);
}

void handleStream() {
    WiFiClient client = server.client();
    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: ");
    client.print(_STREAM_CONTENT_TYPE);
    client.print("\r\n\r\n");

    while (client.connected()) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
            delay(100);
            continue;
        }

        client.print(_STREAM_BOUNDARY);
        char buf[128];
        int len = sprintf(buf, _STREAM_PART, fb->len);
        client.write((const uint8_t *)buf, len);
        client.write(fb->buf, fb->len);
        client.print("\r\n");

        esp_camera_fb_return(fb);
        delay(60); // Stream pacing
    }
}

void handleCapture() {
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) s->set_quality(s, 6); 
    delay(150); 

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Capture Failed");
        if (s != NULL) s->set_quality(s, 14);
        return;
    }

    server.setContentLength(fb->len);
    server.sendHeader("Content-Disposition", "attachment; filename=\"snapshot.jpg\"");
    server.send(200, "image/jpeg", "");
    
    WiFiClient client = server.client();
    client.write(fb->buf, fb->len);

    esp_camera_fb_return(fb);
    if (s != NULL) s->set_quality(s, 14);
}

// Function to transition and start Access Point Mode
void setupAPMode() {
    if (apStarted) return;
    
    Serial.println("No ESP-NOW peer found. Starting Access Point mode...");
    
    WiFi.mode(WIFI_AP_STA);
    
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    
    WiFi.softAP("ESP32S3_CAM_AP", "");
    
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
    
    server.on("/", handleRoot);
    server.on("/stream", handleStream);
    server.on("/capture", handleCapture);
    server.begin();
    
    Serial.println("Web server started.");
    apStarted = true;
}

// ----------------------------------------------------
// ESP-NOW Receive Callback
// ----------------------------------------------------
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    const uint8_t *mac = info->src_addr;
#else
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
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
            currentMode = MODE_ESPNOW;
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
    WiFi.setSleep(false); 
    
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_get_mac(WIFI_IF_STA, cam_mac);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
    Serial.println("Waiting for PyController to broadcast 'pyCAR_DISCOVER'...");
    
    // Save startup time reference to enforce fallback timeout
    startWaitTime = millis();

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

    // CRITICAL FIX: Down-clocked to 10MHz. 20MHz overflows the camera's DMA FIFO buffer causing 
    // it to randomly inject corrupt bytes into the JPEG, destroying the TJpgDec decoder on the controller.
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_QVGA; 
    
    // Loosened compression slightly to ensure hardware consistency.
    config.jpeg_quality = 14; 
    config.fb_count     = 2;  
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera Init Failed");
        return;
    }
    Serial.println("Camera initialized!");

    // --- 3. Flip the Image Hardware-Side ---
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_vflip(s, 1);   
        s->set_hmirror(s, 1); 
    }
}

// Helper function to keep the loop clean
void send_raw_image(camera_fb_t *fb) {
    uint8_t raw_packet[1400];
    
    // Keeps a running sequence number for all sent frames
    static uint16_t seq_num = 0;
    
    // 802.11 MAC Header (24 bytes)
    raw_packet[0] = 0x08; raw_packet[1] = 0x00;
    raw_packet[2] = 0x00; raw_packet[3] = 0x00;
    memcpy(&raw_packet[4], pyControllerMac, 6);  // Addr1 (Dest)
    memcpy(&raw_packet[10], cam_mac, 6);         // Addr2 (Src)
    memcpy(&raw_packet[16], pyControllerMac, 6); // Addr3 (BSSID)

    // Custom Payload Header Setup
    raw_packet[24] = 'C'; raw_packet[25] = 'A'; raw_packet[26] = 'M';
    
    int max_payload = 1300; 
    uint16_t total_chunks = (fb->len + max_payload - 1) / max_payload;
    
    for (uint16_t i = 0; i < total_chunks; i++) {
        
        // CRITICAL FIX: Inject valid incrementing 802.11 Sequence Numbers!
        // The Wi-Fi MAC layer natively filters out duplicate packets. If every packet
        // carries a "0x0000" sequence number, the hardware intercepts it as a network re-transmission
        // and randomly drops your chunks, breaking the JPEG formatting!
        uint16_t seq_ctrl = (seq_num++) << 4; 
        raw_packet[22] = seq_ctrl & 0xFF;
        raw_packet[23] = (seq_ctrl >> 8) & 0xFF;

        memcpy(&raw_packet[27], &i, 2);
        memcpy(&raw_packet[29], &total_chunks, 2);
        
        int offset = i * max_payload;
        uint16_t len = fb->len - offset;
        if (len > max_payload) len = max_payload;
        
        memcpy(&raw_packet[31], &len, 2);
        memcpy(&raw_packet[33], fb->buf + offset, len);
        
        esp_wifi_80211_tx(WIFI_IF_STA, raw_packet, 33 + len, false);
        
        // Increased delay slightly to strictly enforce a safe streaming cadence 
        delayMicroseconds(2000); 
    }
}

void loop() {
    // If waiting for a peer and timeout occurs, switch to Access Point mode
    if (currentMode == MODE_WAITING) {
        if (millis() - startWaitTime > ESPNOW_TIMEOUT_MS) {
            currentMode = MODE_AP;
            setupAPMode();
        }
    }

    if (currentMode == MODE_ESPNOW) {
        // If an ESP-NOW peer is found, ensure AP mode is disabled
        if (apStarted) {
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_STA);
            apStarted = false;
            Serial.println("ESP-NOW connected. Access Point disabled.");
        }

        if (isConnected) {
            if (captureRequested) {
                captureRequested = false;
                
                sensor_t * s = esp_camera_sensor_get();
                if (s != NULL) s->set_quality(s, 6); 
                delay(50); 
                
                camera_fb_t *fb = esp_camera_fb_get();
                if (fb) {
                    send_raw_image(fb);
                    esp_camera_fb_return(fb);
                }
                
                if (s != NULL) s->set_quality(s, 14); 
                
            } else if (is_streaming) {
                camera_fb_t *fb = esp_camera_fb_get();
                if (fb) {
                    send_raw_image(fb);
                    esp_camera_fb_return(fb);
                }
            }
        }
    } else if (currentMode == MODE_AP) {
        // Serve clients in AP Mode
        server.handleClient();
    }
    
    delay(5);
}