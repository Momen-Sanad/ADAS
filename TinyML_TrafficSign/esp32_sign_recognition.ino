#include "esp_camera.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "model_data.h"

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

tflite::MicroErrorReporter tflErrorReporter;
const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;

constexpr int tensorArenaSize = 128 * 1024;
byte tensorArena[tensorArenaSize];

const char* labels[] = {"LEFT", "RETURN", "RIGHT", "SLOW", "STOP"};

void setup() {
  Serial.begin(9600);
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;

  config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM; config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565; 
  config.frame_size = FRAMESIZE_96X96;    
  config.fb_count = 1;

  esp_camera_init(&config);

  tflModel = tflite::GetModel(g_traffic_sign_model); // g_traffic_sign_model is in model_data.h
  static tflite::AllOpsResolver tflResolver;
  static tflite::MicroInterpreter static_interpreter(tflModel, tflResolver, tensorArena, tensorArenaSize, &tflErrorReporter);
  tflInterpreter = &static_interpreter;
  tflInterpreter->AllocateTensors();
  tflInputTensor = tflInterpreter->input(0);
}

void loop() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  for (int i = 0; i < 96 * 96 * 3; i++) {
    tflInputTensor->data.int8[i] = (int8_t)(((float)fb->buf[i] / 255.0f - 0.5f) * 2.0f); // Quantized math
  }
  
  esp_camera_fb_return(fb);

  tflInterpreter->Invoke();

  int top_idx = 0;
  int8_t top_score = -128;
  for (int i = 0; i < 5; i++) {
    if (tflInterpreter->output(0)->data.int8[i] > top_score) {
      top_score = tflInterpreter->output(0)->data.int8[i];
      top_idx = i;
    }
  }

  if (top_score > 80) {
      char cmd = ' ';
      switch(top_idx) {
          case 0: cmd = 'L'; break; // LEFT
          case 1: cmd = 'U'; break; // RETURN (U-Turn)
          case 2: cmd = 'R'; break; // RIGHT
          case 3: cmd = 'D'; break; // SLOW DOWN
          case 4: cmd = 'S'; break; // STOP
      }
      Serial.write(cmd); // Send to ATmega328A
  }

  delay(200);
}
