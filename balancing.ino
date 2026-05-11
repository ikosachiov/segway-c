/**
 * @author Sergey Royz (zjor.se@gmail.com) 
 * @version 0.3 (MPU6050 gravity & scaling fix)
 * @date 2026-05-11
 */

// The MPU 6050 is positioned so that the X-axis is directed backward,
// the Y-axis is directed to the right, and the Z-axis is directed upward.
// The positive direction of rotation along the axes is clockwise.

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include "pid.h"
#include "stepper.h"

#define PIN_IMU_SCL 22
#define PIN_IMU_SDA 21
#define PIN_IMU_INT 23

#define PIN_MOTOR_RIGHT_EN    0 // always en
#define PIN_MOTOR_RIGHT_DIR   33
#define PIN_MOTOR_RIGHT_STEP  25

#define PIN_MOTOR_LEFT_EN    0 // always en
#define PIN_MOTOR_LEFT_DIR   27
#define PIN_MOTOR_LEFT_STEP  26

#define PPR       1600

#define CPU_FREQ_MHZ  240
#define CPU_FREQ_DIVIDER  120
#define TICKS_PER_SECOND (200000 * (CPU_FREQ_MHZ / CPU_FREQ_DIVIDER))
#define PULSE_WIDTH      1

#define MAX_ACCEL (200)
#define ANGLE_Kp  800.0
#define ANGLE_Kd  60.0 
#define ANGLE_Ki  0.0

#define VELOCITY_Kp  0.0005
#define VELOCITY_Kd  0.000005
#define VELOCITY_Ki  0.000002


#define WARMUP_DELAY_US (7000000UL)
#define ANGLE_SET_POINT -0.08

#define LOG_IMU
#define LOG_ENABLED
#define LOG_MS 250

void initTimerInterrupt();
float normalizeAngle(float);
void updateVelocity(unsigned long);
void updateControl(unsigned long);
void log(unsigned long);

Stepper leftStepper(PIN_MOTOR_LEFT_EN, PIN_MOTOR_LEFT_DIR, PIN_MOTOR_LEFT_STEP, TICKS_PER_SECOND, PPR, PULSE_WIDTH);
Stepper rightStepper(PIN_MOTOR_RIGHT_EN, PIN_MOTOR_RIGHT_DIR, PIN_MOTOR_RIGHT_STEP, TICKS_PER_SECOND, PPR, PULSE_WIDTH);

// MPU6050 objects
MPU6050 mpu;
volatile bool dmpDataReady = false;
uint8_t devStatus;           
uint8_t fifoBuffer[64];            
float roll, pitch, yaw;

// DMP specific variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container

void dmpISR() {
  dmpDataReady = true;
}

void initIMU() {
  pinMode(PIN_IMU_INT, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1) {
      Serial.println("MPU6050 connection failed");
      delay(5000);
    }
  }

  devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    while (1) {
      Serial.print("DMP Initialization failed (code ");
      Serial.print(devStatus);
      Serial.println(")");
      delay(5000);
    }
  }

  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), dmpISR, RISING);
  mpu.setIntEnabled(true);
}

bool readIMU() {
  if (dmpDataReady) {
    dmpDataReady = false;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      yaw   = ypr[0];
      pitch = ypr[1];
      roll  = ypr[2];
      return true;
    }
  }
  return false;
}

void logIMU() {
  #ifdef LOG_IMU
  static unsigned long lastTimestamp = millis();
  unsigned long now = millis();
  if (now - lastTimestamp > LOG_MS) {
    // Serial.print("p:"); 
    // Serial.println(pitch);    
    lastTimestamp = now;
  }
  #endif
}

PID anglePID(ANGLE_Kp, ANGLE_Kd, ANGLE_Ki, ANGLE_SET_POINT);
PID velocityPID(VELOCITY_Kp, VELOCITY_Kd, VELOCITY_Ki, 0.0);
bool isBalancing = false;

float accel = 0.0f;
float velocity = 0.0f; 
float angle = 0.0f;
float targetAngle = ANGLE_SET_POINT;

hw_timer_t * timer = NULL;

void IRAM_ATTR onTimer() {
  leftStepper.tick();
  rightStepper.tick();
}

float normalizeAngle(float value) { 
  // DMP output via dmpGetYawPitchRoll is already in Radians (-PI to PI)
  return value;
}

void setupBalancing(void) {
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(1000000UL);

  Serial.println("Start");
  initTimerInterrupt();
  initIMU();

  leftStepper.init();
  rightStepper.init();
  leftStepper.setEnabled(true);
  rightStepper.setEnabled(true);
}

void initTimerInterrupt() {
  timer = timerBegin(666667);                
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 5, true, 0);             
  timerStart(timer);
}

void updateVelocity(unsigned long nowMicros) {
  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 50) {
    return;
  }

  float dt = ((float) (nowMicros - timestamp)) * 1e-6;
  velocity += accel * dt;

  // Drive both wheels together to move straight in response to the lean
  leftStepper.setVelocity(-velocity);
  rightStepper.setVelocity(velocity); 
  
  timestamp = nowMicros;
}

void setBalancing(bool balancing) {
  if (isBalancing != balancing) {
    isBalancing = balancing;
    #ifdef LOG_ENABLED
    Serial.print("IsBalancing: ");
    Serial.println(isBalancing);
    #endif
  }  
}

void setIMUWarmUpElapsed() {
  static bool invoked = false;
  if (!invoked) {
    invoked = true;
    #ifdef LOG_ENABLED
    Serial.println("IMU Warm Up timeout elapsed");
    #endif
  }
}

void updateControl(unsigned long nowMicros) {
  static unsigned long timestamp = micros();
  if (nowMicros - timestamp < 1000) {
    return;
  }

  if (!readIMU()) {
    return;
  }

  if (nowMicros < WARMUP_DELAY_US) {    
    // Keeping system components completely cleared during filter stabilization
    accel = 0.0f;
    velocity = 0.0f;
    leftStepper.setVelocity(0.0f);
    rightStepper.setVelocity(0.0f);
    return;
  } 
  setIMUWarmUpElapsed();

  angle = normalizeAngle(pitch);
  float dt = ((float) (nowMicros - timestamp)) * 1e-6;

  if (abs(angle - targetAngle) < PI / 18) {
    setBalancing(true);
  }

  if (abs(angle - targetAngle) > PI / 4) {
    setBalancing(false);
    accel = 0.0f;
    velocity = 0.0f;    
  }

  if (!isBalancing) {
    return;
  }
  
  targetAngle = ANGLE_SET_POINT + velocityPID.getControl(velocity, dt);
  anglePID.setTarget(targetAngle);
  // anglePID.setTarget(ANGLE_SET_POINT);

  accel = -anglePID.getControl(angle, dt);
  accel = constrain(accel, -MAX_ACCEL, MAX_ACCEL);

  timestamp = nowMicros;
}

void log(unsigned long nowMicros) {  
  static unsigned long timestamp = micros();  
  if (nowMicros - timestamp < 100000) {
    return;
  }
  Serial.print("yaw:");     Serial.print(yaw, 2);
  Serial.print("\tpitch:");   Serial.print(pitch, 2);
  Serial.print("\troll:");    Serial.print(roll, 2);

  Serial.print("\ta0:");     Serial.print(targetAngle, 4);
  Serial.print("\ta:");    Serial.print(angle, 4);
  Serial.print("\tv:");    Serial.print(velocity, 4);
  Serial.print("\tu:");    Serial.println(accel, 4);  
  timestamp = nowMicros;   
}

void taskBalancing(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(500));
    setupBalancing();
    
    // Track execution rate to prevent background starvation
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for(;;) {
        unsigned long nowMicros = micros();
        
        updateVelocity(nowMicros);
        updateControl(nowMicros);
        logIMU();
        
        #ifdef LOG_ENABLED
        log(nowMicros);
        #endif

        // Yield execution to other tasks for exactly 1 FreeRTOS tick (typically 1ms)
        // This stops the ESP32 Watchdog Timer from triggering a crash.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
