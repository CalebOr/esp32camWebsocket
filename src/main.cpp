/* Project based on Inglebard websocket work https://github.com/Inglebard/esp32cam-relay.git */

#include <Arduino.h>
#include "WiFi.h"
#include "esp_camera.h"
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  const char* ssid = "YourSSID";
  const char* password = "YourPassword";
  #define IPSERVER "192.168.0.2"  // IP of the server where you want to send the data
  #define PORTSERVER 2           // Port of the server where you want to send the data
#endif

#define CAMERA_NUMBER_1

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15 
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define LED_RED 33
#define LED_FLASH 4
#include "camera_pins.h"

bool toggleLedRed = false;
bool status = false;

WebSocketsClient webSocket;

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  toggleLedRed = true;
  Serial.println();
  Serial.println(WiFi.localIP());
}// WiFiStationConnected

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.print("-");
  toggleLedRed = false;
  digitalWrite(LED_RED, 0);
  if(status){
    status = false;
  }
  WiFi.reconnect();
}// WiFiStationDisconnected

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WSc] Disconnected!\n");
      digitalWrite(LED_FLASH, 0);
      if(status){
        status = false;
      }
      break;
    case WStype_CONNECTED: {
      Serial.printf("[WSc] Connected to url: %s\n", payload);
      if(!status){
        status = true;
      }
    }
      break;
    case WStype_TEXT:
      if(strcmp((char*)payload, "No clients") == 0){
        digitalWrite(LED_FLASH, 0);
      }else if(strcmp((char*)payload, "Client connected") == 0){
        digitalWrite(LED_FLASH, 1);
      }else{
        Serial.printf("[WSc] get text: %s\n", payload);
      }
      break;
    case WStype_BIN:
      Serial.printf("[WSc] get binary length: %u\n", length);
      break;
    case WStype_PING:
        // pong will be send automatically
        // Serial.printf("[WSc] get ping\n");
        break;
    case WStype_PONG:
        // answer to a ping we send
        // Serial.printf("[WSc] get pong\n");
        break;
    }
}//webSocketEvent

void setupCamera()
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    config.frame_size = FRAMESIZE_VGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 15;
    config.fb_count = 1;
    
    // Init Camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      delay(500);
      ESP.restart();
      return;
    }else{
      Serial.println("Camera init OK");
    }
}

void setup(){
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);    //led RED
  digitalWrite(LED_RED, 0);
  pinMode(LED_FLASH, OUTPUT);    //led FLASH
  digitalWrite(LED_FLASH, 0);
  setupCamera();

  // Connect to Wi-Fi
  WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // server address, port and URL
  #if defined(CAMERA_NUMBER_1)
    webSocket.begin(IPSERVER, PORTSERVER, "/jpgstream_server1");
    Serial.printf("Connecting to %s:%d%s", IPSERVER, PORTSERVER, "/jpgstream_server1");
    Serial.println();
  #elif defined(CAMERA_NUMBER_2)
    webSocket.begin(IPSERVER, PORTSERVER, "/jpgstream_server2");
    Serial.printf("Connecting to %s:%d%s", IPSERVER, PORTSERVER, "/jpgstream_server2");
    Serial.println();
  #elif defined(CAMERA_NUMBER_3)
    webSocket.begin(IPSERVER, PORTSERVER, "/jpgstream_server3");
    Serial.printf("Connecting to %s:%d%s", IPSERVER, PORTSERVER, "/jpgstream_server3");
    Serial.println();
  #endif
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  webSocket.enableHeartbeat(15000, 3000, 2);
}

unsigned long messageTimestamp = 0;
unsigned long ledTimer = 0;
int timeLoops = 0;
uint64_t lastLoop = 0;

void loop() {
    uint64_t now = millis();
    webSocket.loop();
    if(!webSocket.isConnected()){
      return;
    }
    if(now - messageTimestamp > 30) {
        messageTimestamp = now;

        camera_fb_t * fb = NULL;
        
        // Take Picture with Camera
        fb = esp_camera_fb_get();
        if(!fb) {
          Serial.println("Camera capture failed");
          return;
        }
        Serial.printf("Picture taken! Size: %u\n\r", fb->len);
        webSocket.sendBIN(fb->buf, fb->len);
        esp_camera_fb_return(fb);
    }
}