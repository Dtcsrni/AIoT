/* I.S.C. Erick Vega
Sketch para cámara Espcam con identificador de imagenes por IA
 

/* Includes ---------------------------------------------------------------- */
#include <IoT_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include <ArduinoOTA.h>

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM

// Redes Wi-Fi disponibles
const char *ssidPrimaria = "Cybersys_Ext";
const char *passPrimaria = "Sjmahpe122512";

const char *ssidSecundaria = "Cybersys_Tlilkuautli";
const char *passSecundaria = "Sjmahpe122512";

const char *ssidTerciaria = "Cybersys";
const char *passTerciaria = "Sjmahpe122512";

// Variables de control para reconexión WiFi
unsigned long ultimoIntentoWiFi = 0;
const unsigned long intervaloReintentoWiFi = 6666;  // 6.666 segundos entre intentos
int redActual = 0;                                  // 0 = Principal, 1 = Secundaria, 2 = Terciaria


/* Broker MQTT */
const char *servidor_mqtt = "192.168.1.7";
const int puerto_mqtt = 1883;
const char *tema_mqtt = "RN/clasificacion-imagen/ErickVega";
unsigned long ultimoIntentoMQTT = 0;
const unsigned long intervaloReintentoMQTT = 6000;  // 6 segundos entre intentos MQTT

String ultimoLabel = "";                     // Almacena el último label enviado
unsigned long ultimoEnvio = 0;               // Almacena el tiempo del último envío
const unsigned long intervaloEnvio = 15000;  // 6 segundos de intervalo mínimo de envio entre mensajes MQTT del mismo producto


WiFiClient clienteEsp;
PubSubClient cliente(clienteEsp);


//#define CAMERA_MODEL_ESP_EYE  // Has PSRAM
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 4
#define SIOD_GPIO_NUM 18
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 36
#define Y8_GPIO_NUM 37
#define Y7_GPIO_NUM 38
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 35
#define Y4_GPIO_NUM 14
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 34
#define VSYNC_GPIO_NUM 5
#define HREF_GPIO_NUM 27
#define PCLK_GPIO_NUM 25

#elif defined(CAMERA_MODEL_AI_THINKER)
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

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;  // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf;  //points to the output of the capture

static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_JPEG,  //YUV422,GRAYSCALE,RGB565,JPEG
  .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

  .jpeg_quality = 12,  //0-63 lower number means higher quality
  .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);


void conectarWiFi() {
  // Si ya está conectado, salir
  if (WiFi.status() == WL_CONNECTED) return;

  // Esperar al siguiente intento según el intervalo
  if (millis() - ultimoIntentoWiFi < intervaloReintentoWiFi) return;

  ultimoIntentoWiFi = millis();  // Actualiza el tiempo del último intento

  // Seleccionar red según el valor de redActual
  const char *ssid;
  const char *pass;

  switch (redActual) {
    case 0:
      ssid = ssidPrimaria;
      pass = passPrimaria;
      Serial.println("Intentando conectar a la red principal...");
      break;
    case 1:
      ssid = ssidSecundaria;
      pass = passSecundaria;
      Serial.println("Intentando conectar a la red secundaria...");
      break;
    case 2:
      ssid = ssidTerciaria;
      pass = passTerciaria;
      Serial.println("Intentando conectar a la red terciaria...");
      break;
  }

  // Intentar conexión
  WiFi.begin(ssid, pass);

  unsigned long tiempoInicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - tiempoInicio < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConectado a Wi-Fi.");
    Serial.print("Dirección IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nNo se pudo conectar a Wi-Fi.");
    redActual = (redActual + 1) % 3;  // Cambiar a la siguiente red en la lista
  }
}


void conectarMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;  // No hay WiFi, salir

  if (cliente.connected()) return;  // Ya conectado a MQTT, salir

  if (millis() - ultimoIntentoMQTT < intervaloReintentoMQTT) return;  // Aún no es tiempo

  ultimoIntentoMQTT = millis();  // Actualiza el tiempo del último intento
  cliente.setServer(servidor_mqtt, puerto_mqtt);

  Serial.println("Conectando a MQTT...");
  if (cliente.connect("ESP32Cam")) {
    Serial.println("Conectado al broker MQTT.");
  } else {
    Serial.print("Fallo MQTT, rc=");
    Serial.println(cliente.state());
  }
}

/**
* @brief      Arduino setup function
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);            // Configura WiFi en modo estación (cliente)
  cliente.setClient(clienteEsp);  // Vincula el cliente WiFi con MQT
  configurarOTA();                //Configura OTA
  ei_sleep(500);
  //comment out the below line to start inference immediately after upload
  while (!Serial)
    ;
  Serial.println("Edge Impulse Inferencing Demo");
  if (ei_camera_init() == false) {
    ei_printf("Failed to initialize Camera!\r\n");
  } else {
    ei_printf("Camera initialized\r\n");
  }

  ei_printf("\nStarting continious inference in 2 seconds...\n");
  ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop() {
  conectarWiFi();  // Intenta conectar a WiFi
  conectarMQTT();  // Intenta conectar a MQTT si hay WiFi
  cliente.loop();  // Mantiene la conexión MQTT activa

  // Se llama continuamente para gestionar OTA
  ArduinoOTA.handle();

  // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
  if (ei_sleep(5) != EI_IMPULSE_OK) {
    return;
  }

  snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

  // check if allocation was successful
  if (snapshot_buf == nullptr) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
    ei_printf("Failed to capture image\r\n");
    enviarMQTT(tema_mqtt, "fallo", 0);
    free(snapshot_buf);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    enviarMQTT(tema_mqtt, "fallo classifier", 0);
    return;
  }

  // print the predictions
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  ei_printf("Object detection bounding boxes:\r\n");
  for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
    if (bb.value == 0) {
      continue;
    }
    ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
              bb.label,
              bb.value,
              bb.x,
              bb.y,
              bb.width,
              bb.height);
    enviarMQTT(tema_mqtt, bb.label, bb.value);
  }

  // Print the prediction results (classification)
#else
  ei_printf("Predictions:\r\n");
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
    ei_printf("%.5f\r\n", result.classification[i].value);
  }
#endif

  // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
  ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
  ei_printf("Visual anomalies:\r\n");
  for (uint32_t i = 0; i < result.visual_ad_count; i++) {
    ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
    if (bb.value == 0) {
      continue;
    }
    ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
              bb.label,
              bb.value,
              bb.x,
              bb.y,
              bb.width,
              bb.height);
  }
#endif


  free(snapshot_buf);
}

// Función para inicializar OTA
void configurarOTA() {
  ArduinoOTA.onStart([]() {
    String tipo = (ArduinoOTA.getCommand() == U_FLASH)
                    ? "Actualización de firmware"
                    : "Actualización del sistema de archivos";
    Serial.println("Inicio de " + tipo);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nActualización completada.");
  });

  ArduinoOTA.onProgress([](unsigned int progreso, unsigned int total) {
    Serial.printf("Progreso: %u%%\r", (progreso * 100) / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error [%u]: ", error);
    switch (error) {
      case OTA_AUTH_ERROR: Serial.println("Fallo en autenticación"); break;
      case OTA_BEGIN_ERROR: Serial.println("Error al iniciar"); break;
      case OTA_CONNECT_ERROR: Serial.println("Error de conexión"); break;
      case OTA_RECEIVE_ERROR: Serial.println("Error de recepción"); break;
      case OTA_END_ERROR: Serial.println("Error al finalizar"); break;
    }
  });

  ArduinoOTA.begin();
  Serial.println("OTA listo para recibir actualizaciones.");
}

void enviarMQTT(String tema_mqtt, String label, float value) {
  // Verifica si es el mismo label y si no ha pasado el intervalo de 6 segundos
  if ((label == ultimoLabel && millis() - ultimoEnvio < intervaloEnvio)) {
    return;  // Si es el mismo y no ha pasado el tiempo, no enviar
  }

  // Actualiza el último label y el tiempo de envío
  ultimoLabel = label;
  ultimoEnvio = millis();

  // Crear el objeto JSON
  StaticJsonDocument<256> jsonDoc;
  jsonDoc["label"] = label;
  jsonDoc["value"] = value;

  char jsonBuffer[256];
  serializeJson(jsonDoc, jsonBuffer);  // Serializa el objeto JSON

  // Publicar en el tema MQTT
  if (cliente.publish(tema_mqtt.c_str(), jsonBuffer)) {
    Serial.println("Mensaje MQTT enviado:");
    Serial.println(jsonBuffer);
  } else {
    Serial.print("Error al enviar mensaje MQTT: ");
    Serial.println(cliente.state());  // Muestra el estado del cliente para más detalles
  }
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

  if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  //initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, 0);  // lower the saturation
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  s->set_awb_gain(s, 1);
#endif

  is_initialised = true;
  return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

  //deinitialize the camera
  esp_err_t err = esp_camera_deinit();

  if (err != ESP_OK) {
    ei_printf("Camera deinit failed\n");
    return;
  }

  is_initialised = false;
  return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  bool do_resize = false;

  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

  esp_camera_fb_return(fb);

  if (!converted) {
    ei_printf("Conversion failed\n");
    return false;
  }

  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
      || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
    do_resize = true;
  }

  if (do_resize) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height);
  }


  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  // we already have a RGB888 buffer, so recalculate offset into pixel index
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    // Swap BGR to RGB here
    // due to https://github.com/espressif/esp32-camera/issues/379
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  // and done!
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
