#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Define custom ESP32 pins
const int sdaPin = 18;
const int sclPin = 19;

// Create the Adafruit sensor instance
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setupLine() {
  vTaskDelay(pdMS_TO_TICKS(500));
  // Initialize Serial interface
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
  // Parameters: (i2c_address, debug_mode, *theWireBus)
  // 0x29 is the default VL53L0X I2C address, false disables internal library debugging
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
    lox.rangingTest(&measure, false); // Pass 'true' instead of 'false' to dump debugging data
    distance = constrain(measure.RangeMilliMeter, 20, 2000);
    Serial.print("Distance: ");
    Serial.print(measure.RangeMilliMeter);

    // digitalWrite(LED_BUILTIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
    // vTaskDelay(pdMS_TO_TICKS(distance));                    // wait for a second
    // digitalWrite(LED_BUILTIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
    // vTaskDelay(pdMS_TO_TICKS(distance));  

    vTaskDelay(pdMS_TO_TICKS(50));  
  }
}