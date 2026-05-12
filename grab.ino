
const int servoPin = 17;  // GPIO pin

// PWM settings
const int frequency = 50;     // SG90 needs 50Hz
const int resolution = 14;    // 14-bit resolution (0 to 16383)

// SG90 Pulse widths converted to 14-bit duty cycle values
// 50Hz period = 20ms. 16383 units = 20ms.
const int minDuty = 410;   // 0.5ms pulse (0 degrees) -> (0.5/20)*16383
const int maxDuty = 2500;  // 2.4ms pulse (180 degrees) -> (2.4/20)*16383


void setupGrab() {
  ledcAttach(servoPin, frequency, resolution);
}

void taskGrab(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(500));
  setupGrab();

  for (int angle = 0; angle <= 180; angle += 1) {
    int dutyCycle = map(angle, 0, 180, minDuty, maxDuty);
    // In Core 3.0+, ledcWrite directly takes the PIN, not the channel
    ledcWrite(servoPin, dutyCycle);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  
  // Sweep from 180 to 0 degrees
  for (int angle = 180; angle >= 0; angle -= 1) {
    int dutyCycle = map(angle, 0, 180, minDuty, maxDuty);
    ledcWrite(servoPin, dutyCycle);
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  
  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
    vTaskDelay(pdMS_TO_TICKS(300));                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
    vTaskDelay(pdMS_TO_TICKS(300)); 


  }
}