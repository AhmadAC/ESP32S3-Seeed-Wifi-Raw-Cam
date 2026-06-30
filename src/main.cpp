// ESP32S3-Seeed-Wifi-Raw-Cam/src/main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <esp_idf_version.h>
#include "esp_camera.h"
#include <WebServer.h>
#include <Preferences.h>

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

// State management for ESP-NOW and Wi-Fi networks
enum SystemMode {
    MODE_WAITING,
    MODE_ESPNOW,
    MODE_STA,
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

// Reusable flat JSON extractor for POST requests
String getJsonValue(const String& json, const String& key) {
    int keyIndex = json.indexOf("\"" + key + "\"");
    if (keyIndex == -1) return "";
    int colonIndex = json.indexOf(":", keyIndex);
    if (colonIndex == -1) return "";
    int startQuote = json.indexOf("\"", colonIndex);
    if (startQuote == -1) return "";
    int endQuote = json.indexOf("\"", startQuote + 1);
    if (endQuote == -1) return "";
    return json.substring(startQuote + 1, endQuote);
}

// HTML, CSS, and JS interface served to connected phones/devices
const char SETUP_HTML[] PROGMEM = R"raw_html(
<!DOCTYPE html><html><head><meta charset="utf-8"><title>ESP Setup</title><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    :root { --primary: #0ea5e9; --bg: #0f172a; --card: #1e293b; --text: #f1f5f9; }
    body { font-family: -apple-system, sans-serif; background: var(--bg); color: var(--text); padding: 15px; display: flex; flex-direction: column; align-items: center; min-height: 100vh; margin:0;}
    .container { width: 100%; max-width: 420px; }
    .card { background: var(--card); padding: 25px; border-radius: 20px; border: 1px solid #334155; text-align: center; margin-top: 15px; }
    h2 { color: var(--primary); margin-top: 0; }
    input, select { width: 100%; padding: 12px; margin: 8px 0 20px; border-radius: 10px; border: 1px solid #475569; background: #0f172a; color: white; box-sizing: border-box; }
    button { width: 100%; padding: 15px; border: none; border-radius: 12px; font-weight: bold; cursor: pointer; color: white; margin-top:10px; transition: transform 0.1s; }
    button:active { transform: scale(0.96); opacity: 0.9; }
    .btn-green { background: #10b981; }
    .btn-red { background: #ef4444; }
    .btn-blue { background: #3b82f6; }
    .status-bar { padding: 12px; border-radius: 10px; font-weight: bold; text-align: center; font-size: 0.85rem; border: 1px solid; text-transform: uppercase; background: #451a03; color: #fbbf24; border-color: #f59e0b; }
</style></head>
<body>
    <div class="container">
        <div class="status-bar">WIFI SETUP MODE</div>
        <div class="card">
            <h2>WiFi Setup</h2>
            <div id="status-msg" style="font-size:0.8rem;color:#64748b;margin-bottom:5px">Ready to Scan</div>
            <button class="btn-green" onclick="scan()">Scan Networks</button>
            <select id="ssid" style="margin-top:10px;"><option value="">-- Select --</option></select>
            <input type="password" id="pass" placeholder="Password">
            <button class="btn-green" onclick="save()">Save and Reboot</button>
            <button class="btn-blue" style="margin-top:15px;" onclick="location.href='/app'">Skip to Live Stream</button>
            <button class="btn-red" style="margin-top:15px;" onclick="resetData()">Factory Reset Device</button>
        </div>
    </div>
    <script>
    function scan(){
        document.getElementById('status-msg').innerText="Scanning...";
        fetch('/scan').then(r=>{
            if(!r.ok) throw new Error("Server returned error");
            return r.json();
        }).then(d=>{
            const s=document.getElementById('ssid'); s.innerHTML='<option value="">-- Select --</option>';
            d.forEach(n=>{let o=document.createElement('option');o.value=n;o.innerText=n;s.appendChild(o)});
            document.getElementById('status-msg').innerText="Networks Found: " + d.length;
        }).catch(()=>{ document.getElementById('status-msg').innerText="Scan Error"; });
    }
    function save(){
        const s=document.getElementById('ssid').value, p=document.getElementById('pass').value;
        if(!s) return alert('Select SSID');
        fetch('/save', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ssid: s, pass: p}) })
        .then(r=>r.text()).then(t=>{ alert('Saved! Rebooting...'); });
    }
    function resetData(){
        if(confirm("Are you sure?")) fetch('/reset', { method: 'POST' }).then(() => alert('Resetting...'));
    }
    </script>
</body></html>
)raw_html";

const char APP_HTML[] PROGMEM = R"raw_html(
<!DOCTYPE html><html><head><meta charset="utf-8"><title>Camera Live Stream</title><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
    :root { --primary: #0ea5e9; --bg: #0f172a; --card: #1e293b; --text: #f1f5f9; }
    body { font-family: -apple-system, sans-serif; background: var(--bg); color: var(--text); padding: 15px; display: flex; flex-direction: column; align-items: center; min-height: 100vh; margin:0;}
    .container { width: 100%; max-width: 600px; }
    .card { background: var(--card); padding: 25px; border-radius: 20px; border: 1px solid #334155; text-align: center; margin-top: 15px; }
    h1 { color: #00adb5; margin-bottom: 20px; }
    .stream-container { width: 100%; border-radius: 8px; overflow: hidden; background-color: #000; margin-bottom: 20px; position: relative; }
    img { display: block; width: 100%; height: auto; }
    button { width: 100%; padding: 15px; border: none; border-radius: 12px; font-weight: bold; cursor: pointer; color: white; transition: transform 0.1s; }
    button:active { transform: scale(0.96); opacity: 0.9; }
    .btn-blue { background: #3b82f6; margin-top:10px; }
    .btn-gray { background: #475569; }
    .status-bar { padding: 12px; border-radius: 10px; font-weight: bold; text-align: center; font-size: 0.85rem; border: 1px solid; text-transform: uppercase; background: #172554; color: #93c5fd; border-color: #3b82f6; }
    .status-text { margin-top: 10px; color: #aaaaaa; font-size: 14px; }
</style>
</head>
<body>
    <div class="container">
        <div class="status-bar">CAMERA LIVE STREAM</div>
        <div class="card">
            <div class="stream-container">
                <img src="/stream" alt="Live Stream">
            </div>
            <button id="snapBtn" class="btn-blue" onclick="takeSnapshot()">Save Snapshot</button>
            <div id="status" class="status-text">Streaming live...</div>
            <button class="btn-gray" style="margin-top:20px; background:#0f172a; border:1px solid #475569;" onclick="location.href='/setup'">Go to Wi-Fi Setup</button>
        </div>
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
</body></html>
)raw_html";

// Web server request handlers
void handleRoot() {
    server.sendHeader("Location", (currentMode == MODE_AP) ? "/setup" : "/app", true);
    server.send(302, "text/plain", "");
}

void handleSetup() {
    server.send(200, "text/html", SETUP_HTML);
}

void handleApp() {
    server.send(200, "text/html", APP_HTML);
}

// FIX: Gracefully catch 404s and automatically dismiss /favicon.ico spam in the serial console
void handleNotFound() {
    if (server.uri() == "/favicon.ico") {
        server.send(204, "image/x-icon", ""); // Empty response stops the browser from trying again
        return;
    }
    server.send(404, "text/plain", "Not Found");
}

void handleScan() {
    int n = WiFi.scanNetworks();
    
    // FIX: Properly handle and return HTTP 500 if the scan fails, so the web UI catches it
    if (n < 0) {
        server.send(500, "text/plain", "Scan Failed");
        return;
    }
    
    String json = "[";
    for (int i = 0; i < n; ++i) {
        json += "\"" + WiFi.SSID(i) + "\"";
        if (i < n - 1) json += ",";
    }
    json += "]";
    server.send(200, "application/json", json);
}

void handleSave() {
    if (server.hasArg("plain")) {
        String body = server.arg("plain");
        String ssid = getJsonValue(body, "ssid");
        String pass = getJsonValue(body, "pass");
        
        if (ssid.length() > 0) {
            Preferences prefs;
            prefs.begin("storage", false);
            prefs.putString("wifi_ssid", ssid);
            prefs.putString("wifi_pass", pass);
            prefs.end();
            Serial.println("Saved Wi-Fi credentials to NVS");
            
            server.send(200, "text/plain", "OK");
            delay(1000);
            ESP.restart();
            return;
        }
    }
    server.send(400, "text/plain", "Bad Request");
}

void handleReset() {
    Preferences prefs;
    prefs.begin("storage", false);
    prefs.clear();
    prefs.end();
    Serial.println("NVS Erased. Factory reset complete.");
    
    server.send(200, "text/plain", "OK");
    delay(1000);
    ESP.restart();
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
    if (s != NULL) {
        s->set_framesize(s, FRAMESIZE_UXGA); // Temporarily increase frame size to full 1600x1200
        s->set_quality(s, 10);              // Boost image quality
    }
    
    // Crucial step: Clear stale low-res frames in transit queue
    for (int i = 0; i < 4; i++) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
        delay(30);
    }

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Capture Failed");
        if (s != NULL) {
            s->set_framesize(s, FRAMESIZE_QVGA);
            s->set_quality(s, 14);
        }
        return;
    }

    server.sendHeader("Content-Disposition", "attachment; filename=\"snapshot.jpg\"");
    server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);

    esp_camera_fb_return(fb);

    // Revert camera settings back to low resolution for smooth live stream/ESP-NOW
    if (s != NULL) {
        s->set_framesize(s, FRAMESIZE_QVGA);
        s->set_quality(s, 14);
    }
}

// Set up server route maps
void startWebServerHandlers() {
    server.on("/", handleRoot);
    server.on("/setup", handleSetup);
    server.on("/app", handleApp);
    server.on("/scan", handleScan);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/reset", HTTP_POST, handleReset);
    server.on("/stream", handleStream);
    server.on("/capture", handleCapture);
    
    // Catch-all to elegantly dismiss missing routes and favicons
    server.onNotFound(handleNotFound);
    
    server.begin();
    Serial.println("Web server started.");
}

// Function to handle connection or AP setup
void setupWiFi() {
    Preferences prefs;
    prefs.begin("storage", true);
    String ssid = prefs.getString("wifi_ssid", "");
    String pass = prefs.getString("wifi_pass", "");
    prefs.end();

    if (ssid.length() > 0) {
        Serial.print("Connecting to saved Wi-Fi network: ");
        Serial.println(ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), pass.c_str());
        
        // Wait up to 12 seconds for station connection
        unsigned long startConnect = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startConnect < 12000) {
            delay(500);
            Serial.print(".");
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nSuccessfully connected to Wi-Fi!");
            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());
            currentMode = MODE_STA;
            startWebServerHandlers();
            return;
        } else {
            Serial.println("\nConnection to router timed out.");
        }
    } else {
        Serial.println("No saved Wi-Fi credentials found.");
    }
    
    // Fallback to AP Mode
    Serial.println("Starting fallback Access Point...");
    
    // FIX: Set to WIFI_AP_STA (Access Point + Station). 
    // Station mode MUST be active in the background for WiFi.scanNetworks() to successfully scan routers!
    WiFi.mode(WIFI_AP_STA);
    
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    
    WiFi.softAP("ESP32S3_CAM_AP", "");
    Serial.print("AP IP Address: ");
    Serial.println(WiFi.softAPIP());
    
    currentMode = MODE_AP;
    apStarted = true;
    startWebServerHandlers();
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

    // Allocate memory for high resolution frames from the start
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_UXGA; // Crucial for reserving memory block for high-res snapshots
    
    // Loosened compression slightly to ensure hardware consistency.
    config.jpeg_quality = 14; 
    config.fb_count     = 2;  
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera Init Failed");
        return;
    }
    Serial.println("Camera initialized!");

    // --- 3. Configure defaults and dynamic resolution ---
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_vflip(s, 1);   
        s->set_hmirror(s, 1); 
        s->set_framesize(s, FRAMESIZE_QVGA); // Default back down to QVGA for light streaming
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
    // If waiting for a peer and timeout occurs, setup local connection or start AP fallback
    if (currentMode == MODE_WAITING) {
        if (millis() - startWaitTime > ESPNOW_TIMEOUT_MS) {
            setupWiFi();
        }
    }

    if (currentMode == MODE_ESPNOW) {
        // If ESP-NOW peer connects, ensure AP or station mode is turned off
        if (apStarted || WiFi.getMode() != WIFI_STA) {
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_STA);
            apStarted = false;
            Serial.println("ESP-NOW active. Access Point server disabled.");
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
    } else if (currentMode == MODE_STA || currentMode == MODE_AP) {
        // Handle incoming client requests in Router or AP Web Server mode
        server.handleClient();
    }
    
    delay(5);
}