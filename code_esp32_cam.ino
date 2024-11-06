// This code is used to receive signals from Blynk or the phone application (optional: using distance sensor) to take photos and send photos to the AI ​​server
// If the response status is OK, send a signal to Blynk so that BLynk can send it to ESP32 to open the door (servo)
// If the response status is ERR, send a signal to Blynk so that BLynk can send it to ESP32 to warn (buzzer) and the app
#define BLYNK_TEMPLATE_ID "YOUR_BLYNK_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "YOUR_BLYNK_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN "YOUR_BLYNK_AUTH_TOKEN"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ArduinoJson.h>

char ssid[] = ""; //Your ssid
char pass[] = ""; //Your pass




String serverName = "";   //Your Sever
String serverPathForSend = "/api/detection/send-image";   
String serverPathForWarning = "/api/detection/warning";     
const int serverPort = ; // Your port server

WiFiClient client;
// Semaphore để giới hạn số yêu cầu chụp ảnh và gửi ảnh cùng lúc
SemaphoreHandle_t sendPhotoSemaphore;

#define MAX_CONCURRENT_PHOTO_REQUESTS 3 // Giới hạn 3 yêu cầu chụp và gửi ảnh

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const int timerInterval = 30000;
unsigned long previousMillis = 0;

BLYNK_WRITE(V0) {
  int pinValue = param.asInt();
  if (pinValue == 1) {
    sendPhoto();  // Gọi hàm chụp và gửi ảnh
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

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

  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 5; 
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 5; 
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // Tạo Counting Semaphore với giới hạn tối đa
  sendPhotoSemaphore = xSemaphoreCreateCounting(MAX_CONCURRENT_PHOTO_REQUESTS, MAX_CONCURRENT_PHOTO_REQUESTS);
}

String sendPhoto() {
  // Chờ nếu đã đạt giới hạn số yêu cầu đang được xử lý
  if (xSemaphoreTake(sendPhotoSemaphore, portMAX_DELAY) == pdTRUE) {
    String getAll;
    String getBody;

    camera_fb_t *fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      delay(1000);
      ESP.restart();
    }

    Serial.println("Connecting to server: " + serverName);

    if (client.connect(serverName.c_str(), serverPort)) {
      Serial.println("Connection successful!");
      String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
      String tail = "\r\n--RandomNerdTutorials--\r\n";

      uint32_t imageLen = fb->len;
      uint32_t extraLen = head.length() + tail.length();
      uint32_t totalLen = imageLen + extraLen;

      client.println("POST " + serverPathForSend + " HTTP/1.1");
      client.println("Host: " + serverName);
      client.println("Content-Length: " + String(totalLen));
      client.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
      client.println();
      client.print(head);

      uint8_t *fbBuf = fb->buf;
      size_t fbLen = fb->len;
      for (size_t n = 0; n < fbLen; n += 1024) {
        if (n + 1024 < fbLen) {
          client.write(fbBuf, 1024);
          fbBuf += 1024;
        } else if (fbLen % 1024 > 0) {
          size_t remainder = fbLen % 1024;
          client.write(fbBuf, remainder);
        }
      }
      client.print(tail);

      esp_camera_fb_return(fb);

      int timeoutTimer = 10000;
      long startTimer = millis();
      boolean state = false;

      while ((startTimer + timeoutTimer) > millis()) {
        Serial.print(".");
        delay(100);
        while (client.available()) {
          char c = client.read();
          if (c == '\n') {
            if (getAll.length() == 0) { state = true; }
            getAll = "";
          } else if (c != '\r') { getAll += String(c); }
          if (state == true) { getBody += String(c); }
          startTimer = millis();
        }
        if (getBody.length() > 0) { break; }
      }
      Serial.println();
      client.stop();
      Serial.println(getBody);

      StaticJsonDocument<1024> doc;
      DeserializationError error = deserializeJson(doc, getBody);

      if (error) {
        Serial.println("Failed to parse JSON");
      } else {
        const char *status = doc["status"];
        if (strcmp(status, "OK") == 0) {
          Serial.println("Status OK - Open door");
          Blynk.virtualWrite(V1, 1);
          delay(1000);
          Blynk.virtualWrite(V1, 0);
        } else {
          Serial.println("Status NOT OK - Warning");
          Blynk.virtualWrite(V2, 1);
          delay(1000);
          Blynk.virtualWrite(V2, 0);
        }
      }
    } else {
      getBody = "Connection to " + serverName + " failed.";
      Serial.println(getBody);
    }

    // Trả lại Semaphore sau khi yêu cầu hoàn thành
    xSemaphoreGive(sendPhotoSemaphore);

    return getBody;
  }
  return "Semaphore Timeout"; // Trả về trong trường hợp không lấy được semaphore (hiếm khi xảy ra do dùng portMAX_DELAY)
}

void loop() {
  Blynk.run();
}
