#include <esp_camera.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#include "config.h"

const int httpPort = 80;

unsigned long xclk = 8;

#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define LED_PIN 33
#define LED_ON LOW
#define LED_OFF HIGH
#define LAMP_PIN 4

AsyncWebServer server(httpPort); // HTTP服务器对象

template <typename Generic>
void DEBUG(Generic text)
{
  Serial.print(F("*Client: "));
  Serial.println(text);
}

void setup()
{
  Serial.begin(115200);
  DEBUG("starting...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    DEBUG("Connecting to WiFi...");
  }

  DEBUG("Connected to WiFi");
  DEBUG("IP Address: ");
  DEBUG(WiFi.localIP());

  // 初始化摄像头
  DEBUG("initializing camera...");
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
  config.xclk_freq_hz = xclk * 1000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  // Pre-allocate large buffers
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // 初始化摄像头
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    DEBUG("camera initialization failed");
    return;
  }
  else
  {
    DEBUG("camera initialized");
  }

  // 设置 Web 服务器路由
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hello, ESP32!"); });

  server.on("/camera", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb) {
      request->send(500, "text/plain", "camera capture failed");
      return;
    }

    request->send_P(200, "image/jpeg", (const uint8_t*)fb->buf, fb->len);
    esp_camera_fb_return(fb); });

  server.begin();
}

void loop()
{
}
