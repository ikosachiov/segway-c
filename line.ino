#include <Wire.h>
#include "pid.h" // Используем встроенный класс PID
#include <AsyncUDP.h>

AsyncUDP udp;

// === НАСТРОЙКИ ДАТЧИКОВ И ПИНОВ ===
const int pinLineLeft = 35;   // Укажи свои аналоговые пины (например, ADC1 пины ESP32)
const int pinLineRight = 34;

// === КОЭФФИЦИЕНТЫ ПИД-РЕГУЛЯТОРА ЛИНИИ ===
const float LINE_Kp = 0.05f;  // Пропорциональный коэффициент (начни с малого: 0.05 - 0.2)
const float LINE_Kd = 0.01f;  // Дифференциальный коэффициент (гасит раскачку)
const float LINE_Ki = 0.00f;  // Интегральный коэффициент (для линии обычно не нужен)

// === МАРШЕВАЯ СКОРОСТЬ ===
const float BASE_VELOCITY = 5.0f; // Базовая скорость робота вперед по линии

// Подключаем переменные управления из balancing.ino
extern volatile float userTargetVelocity;
extern volatile float userTargetSteering;
extern volatile float userLineSteering; // Новая общая переменная для руления по линии

void setupLine() {
  Serial.begin(115200);
  pinMode(pinLineLeft, INPUT);
  pinMode(pinLineRight, INPUT);

  // UDP setup
  udp.listen(8888);

  Serial.println("Line Follower Task Initialized.");
}

void taskLine(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(500));
  setupLine();

  // Инициализируем ПИД. Целевое значение (setpoint) = 0.0 (датчики должны давать одинаковые значения)
  PID linePID(LINE_Kp, LINE_Kd, LINE_Ki, 0.0f);

  unsigned long lastTime = millis();

  int counter = 0;
  for(;;) {
    counter++;
    unsigned long now = millis();
    float dt = (now - lastTime) * 0.001f;
    if (dt <= 0.0f) dt = 0.001f;
    lastTime = now;

    // Читаем показания датчиков (0...4095 для ESP32)
    int leftVal = analogRead(pinLineLeft);
    int rightVal = analogRead(pinLineRight);

    // Считаем ошибку (разницу между левым и правым датчиком)
    // Если робот уходит с линии и поворачивает НЕ В ТУ сторону, поменяй их местами: (rightVal - leftVal)
    float error = (float)(leftVal - rightVal);

    // Расчет управляющего воздействия ПИД
    float control = linePID.getControl(error, dt);

    // Ограничиваем силу руления, чтобы робот не падал при резких поворотах
    control = constrain(control, -8.0f, 8.0f);

    // Передаем данные в систему балансировки
    userTargetVelocity = BASE_VELOCITY; // Задаем движение вперед
    userTargetSteering = 0.0f;          // Старый руль сбрасываем
    userLineSteering = control;         // Передаем поправку от линии

    if (counter % 25 == 0) {
      char buffer[256];            
      snprintf(buffer, 256, "Line Left:%d, Line Right:%d, LineSteering:%.4f, %d", leftVal, rightVal, control, 0);
      Serial.printf("%s\n", buffer);
      udp.broadcast(buffer);
    }

    Serial.print("line left: "); Serial.print(leftVal);
    Serial.print(" line right: "); Serial.println(rightVal);

    // Высокая частота опроса важна для удержания линии (~50 Гц)
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
