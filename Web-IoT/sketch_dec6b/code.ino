#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE // WAJIB ADA untuk modul Gamepad
#include <DabbleESP32.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

// ==========================================
// 1. KONFIGURASI WIFI (UPDATED) & MQTT
// ==========================================
const char* ssid = "iPhone";             // <--- SUDAH DIGANTI
const char* password = "11111111";       // <--- SUDAH DIGANTI
const char* mqtt_server = "broker.hivemq.com"; 
const int mqtt_port = 1883;

// Topik MQTT (Harus sama dengan Web Dashboard)
const char* topic_volt = "ui/iotfinpro/telemetry/voltage";
const char* topic_speed = "ui/iotfinpro/telemetry/speed";     
const char* topic_angle = "ui/iotfinpro/telemetry/steering";  
const char* topic_cmd_speed = "ui/iotfinpro/control/speed";
const char* topic_cmd_steer = "ui/iotfinpro/control/steer";

// ==========================================
// 2. PIN HARDWARE (Sesuai Rangkaian)
// ==========================================
#define IN1 25
#define IN2 26
#define SERVO_PIN 15
#define TRIG_PIN 5
#define ECHO_PIN 18
#define BATTERY_PIN 34  

// ==========================================
// 3. OBJEK & STRUKTUR DATA
// ==========================================
WiFiClient espClient;
PubSubClient client(espClient);
Servo steeringServo;

// Paket Data Kontrol Internal
struct CarState {
  int speedPWM;   // -255 (Mundur) s/d 255 (Maju), 0 (Stop)
  int steerAngle; // 0 - 180
  bool isBluetooth; 
};

QueueHandle_t commandQueue;      
SemaphoreHandle_t dataMutex;     
TimerHandle_t batteryTimer;      
unsigned long lastActivityTime = 0; 

// Variabel Global
float sharedVoltage = 0.0;
int sharedSpeed = 0;
int sharedAngle = 90;

// ==========================================
// 4. FUNGSI & CALLBACK MQTT
// ==========================================
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];

  CarState mqttData;
  mqttData.isBluetooth = false; // Tandai ini dari Web/WiFi
  mqttData.speedPWM = sharedSpeed; 
  mqttData.steerAngle = sharedAngle;

  if (String(topic) == topic_cmd_speed) mqttData.speedPWM = message.toInt();
  else if (String(topic) == topic_cmd_steer) mqttData.steerAngle = message.toInt();

  xQueueOverwrite(commandQueue, &mqttData);
  lastActivityTime = millis();
}

void reconnectMQTT() {
  if (!client.connected()) {
    String clientId = "IoTFinpro-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      client.subscribe(topic_cmd_speed);
      client.subscribe(topic_cmd_steer);
    } 
  }
}

// ==========================================
// 5. TASK DEFINITIONS (FreeRTOS)
// ==========================================

// --- TASK 1: INPUT BLUETOOTH (DABBLE GAMEPAD) ---
void TaskInputBT(void *pvParameters) {
  CarState btData;
  btData.isBluetooth = true;

  for (;;) {
    Dabble.processInput(); 
    
    // --- LOGIKA KONTROL DABBLE ---
    int targetSpeed = 0;
    
    // Gas & Rem
    if (GamePad.isCrossPressed()) {
        targetSpeed = 255;       // X = Maju Full
    } 
    else if (GamePad.isSquarePressed()) {
        targetSpeed = 150;       // Kotak = Maju Santai
    } 
    else if (GamePad.isTrianglePressed()) {
        targetSpeed = -200;      // Segitiga = Mundur
    }
    else {
        targetSpeed = 0;         // Lepas = Stop
    }

    // Setir
    int targetAngle = 90; 
    
    if (GamePad.isLeftPressed()) {
        targetAngle = 50;        // Kiri
    } 
    else if (GamePad.isRightPressed()) {
        targetAngle = 130;       // Kanan
    }

    // Kirim data jika ada tombol ditekan
    if (targetSpeed != 0 || targetAngle != 90 || GamePad.isStartPressed()) { 
        btData.speedPWM = targetSpeed;
        btData.steerAngle = targetAngle;
        xQueueOverwrite(commandQueue, &btData);
        lastActivityTime = millis();
    }
    // Jika lepas semua tombol, kirim STOP sekali saja
    else if (sharedSpeed != 0 || sharedAngle != 90) {
        btData.speedPWM = 0;
        btData.steerAngle = 90;
        xQueueOverwrite(commandQueue, &btData);
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS); 
  }
}

// --- TASK 2: MOTOR & ACTUATOR ---
void TaskActuator(void *pvParameters) {
  CarState currentCmd;
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  steeringServo.attach(SERVO_PIN);
  
  for (;;) {
    if (xQueuePeek(commandQueue, &currentCmd, 0) == pdTRUE) {
      int pwmVal = abs(currentCmd.speedPWM);
      
      if (currentCmd.speedPWM > 0) { // MAJU
        analogWrite(IN1, pwmVal); analogWrite(IN2, 0);
      } 
      else if (currentCmd.speedPWM < 0) { // MUNDUR
        analogWrite(IN1, 0); analogWrite(IN2, pwmVal);
      } 
      else { // STOP
        analogWrite(IN1, 0); analogWrite(IN2, 0);
      }
      steeringServo.write(currentCmd.steerAngle);

      if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
         sharedSpeed = currentCmd.speedPWM;
         sharedAngle = currentCmd.steerAngle;
         xSemaphoreGive(dataMutex);
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS); 
  }
}

// --- TASK 3: TELEMETRY (WiFi) ---
void TaskTelemetry(void *pvParameters) {
  // Hubungkan WiFi dengan parameter baru
  WiFi.begin(ssid, password);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
        if (!client.connected()) reconnectMQTT();
        client.loop(); 

        static unsigned long lastPub = 0;
        if (millis() - lastPub > 500) { 
            lastPub = millis();
            float voltLog; int speedLog; int angleLog;

            if (xSemaphoreTake(dataMutex, (TickType_t) 50) == pdTRUE) {
                voltLog = sharedVoltage; speedLog = sharedSpeed; angleLog = sharedAngle;
                xSemaphoreGive(dataMutex);
            }
            client.publish(topic_volt, String(voltLog).c_str());
            client.publish(topic_speed, String(speedLog).c_str());
            client.publish(topic_angle, String(angleLog).c_str());
        }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); 
  }
}

// --- TASK POWER ---
void batteryCheckCallback(TimerHandle_t xTimer) {
  int adc = analogRead(BATTERY_PIN); 
  float voltage = (adc / 4095.0) * 3.3 * 2; 
  if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) {
    sharedVoltage = voltage; xSemaphoreGive(dataMutex);
  }
}

void TaskPower(void *pvParameters) {
  for (;;) {
    if (millis() - lastActivityTime > 60000) esp_deep_sleep_start();
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  Serial.begin(115200);
  Dabble.begin("ESP32_IoTFinpro"); // Nama Bluetooth
  
  dataMutex = xSemaphoreCreateMutex();
  commandQueue = xQueueCreate(1, sizeof(CarState));
  batteryTimer = xTimerCreate("BatCheck", pdMS_TO_TICKS(1000), pdTRUE, 0, batteryCheckCallback);
  xTimerStart(batteryTimer, 0);

  xTaskCreatePinnedToCore(TaskInputBT, "Input", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskActuator, "Motor", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskTelemetry, "IoT", 4096, NULL, 1, NULL, 0);
  xTaskCreate(TaskPower, "Power", 2048, NULL, 0, NULL);
}

void loop() { vTaskDelete(NULL); }