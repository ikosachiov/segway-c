#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Подключаем переменные управления из balancing.ino
extern volatile float userTargetVelocity;
extern volatile float userTargetSteering;

// Define custom ESP32 pins
const int sdaPin = 18;
const int sclPin = 19;

// Create the Adafruit sensor instance
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setupLine() {
  vTaskDelay(pdMS_TO_TICKS(500));
  
  // Если Serial уже инициализирован в setupBalancing, эту строку можно оставить для надежности, 
  // но лучше следить чтобы оба таска не инитили его одновременно.
  Serial.begin(115200);
  while (!Serial) { delay(1); } // Wait for serial monitor

  Serial.println("\n--- Adafruit VL53L0X ESP32 Test ---");

  // 1. Initialize ESP32 hardware I2C with your exact target pins 18 and 19
  bool i2cInitSuccess = Wire1.begin(sdaPin, sclPin, 100000);
  if (!i2cInitSuccess) {
    Serial.println("Error: Failed to configure ESP32 hardware I2C bus configuration.");
    while (1);
  }

  // 2. Pass the custom configured Wire reference directly into the Adafruit begin method
  if (!lox.begin(0x29, false, &Wire1)) {
    Serial.println(F("Failed to boot VL53L0X sensor. Check your hardware connections."));
    while (1); // Halt execution if sensor initialization fails
  }

  Serial.println(F("VL53L0X sensor successfully configured on Pins 18/19."));
}

void taskLine(void *pvParameters) {
 
  VL53L0X_RangingMeasurementData_t measure;
  int distance = 0;
  setupLine();

  for(;;) {
    lox.rangingTest(&measure, false);
    
    // Если статус измерения не равен 4 (фаза неудачна), то данные корректны
    if (measure.RangeStatus != 4) {
        distance = constrain(measure.RangeMilliMeter, 20, 2000);
        
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" mm");

        // === ЛОГИКА ДВИЖЕНИЯ В ЗАВИСИМОСТИ ОТ РАССТОЯНИЯ ===
        
        if (distance > 400) {
            // Препятствий нет -> едем прямо
            userTargetVelocity = 10.0f;  // <-- Настрой значение скорости
            userTargetSteering = 0.0f;    // Не поворачиваем
        } 
        else if (distance > 150) {
            // Препятствие близко -> останавливаемся и поворачиваем на месте
            userTargetVelocity = 0.0f; 
            userTargetSteering = 5.0f;   // <-- Настрой скорость поворота
        } 
        else {
            // Препятствие в упор -> сдаем назад
            userTargetVelocity = -10.0f; // Отрицательная скорость
            userTargetSteering = 0.0f;
        }

    } else {
        Serial.println("Distance: Out of range");
        
        // Потеряли объект / вне зоны видимости -> стоим на месте
        userTargetVelocity = 0.0f;
        userTargetSteering = 0.0f;
    }

    // Задержка опроса дальномера (влияет на отзывчивость)
    vTaskDelay(pdMS_TO_TICKS(50));  
  }
}