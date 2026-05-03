// Add these includes at the VERY TOP of your file (before anything else)
#include <Wire.h>
#include <AsyncUDP.h>

AsyncUDP udp;


#define STEP0 25
#define DIR0  33
#define STEP1 26
#define DIR1  27

unsigned long lastStepTime = 0;

const float MAX_STEPS_PER_SEC = 2500.0;
const float MIN_STEP_INTERVAL_US = 1000000.0 / MAX_STEPS_PER_SEC;

// REDUCED PID constants for less aggressive balancing
float Kp = 0.15;   // Reduced from 30
float Ki = 0.25;   // Reduced from 0.5
float Kd = 0.25;   // Increased for better damping

// PID variables
float setpoint = 0.0;
float integral = 0.0;
float previous_error = 0.0;
float lastRawAngle = 0.0;
unsigned long lastLoopTime = 0;

//
float lastFilteredAngle = 0.0;

// Complementary filter
float filteredAngle = 0.0;
float gyroRate = 0.0;
unsigned long lastTimestamp = 0;

void setupBalancing() {
    pinMode(STEP0, OUTPUT);
    pinMode(DIR0, OUTPUT);
    pinMode(STEP1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    
    digitalWrite(STEP0, LOW);
    digitalWrite(STEP1, LOW);
}

void doOneStepOrNone(float speed) {
    if (fabs(speed) < 0.005) return;  // Dead zone to prevent jitter
    
    // Set direction
    
    digitalWrite(DIR1, speed > 0 ? HIGH : LOW);
    digitalWrite(DIR0, speed > 0 ? HIGH : LOW);    
    
    // Convert speed to step interval
    float absSpeed = constrain(fabs(speed), 0.05, 0.95);  // Min 0.05 to prevent too slow
    
    // Speed 0.95 = ~2375 steps/sec = 420us interval
    // Speed 0.05 = 125 steps/sec = 8000us interval
    unsigned long stepIntervalUs = (unsigned long)(MIN_STEP_INTERVAL_US / absSpeed);
    
    unsigned long now = micros();

    if (now - lastStepTime >= stepIntervalUs) {
        digitalWrite(STEP1, HIGH);        
        digitalWrite(STEP0, HIGH);        
        delayMicroseconds(25);
        digitalWrite(STEP1, LOW);                
        digitalWrite(STEP0, LOW);
        delayMicroseconds(25);
        lastStepTime = now;
    }
}

    float getAngle() {
        SensorData receivedData;
        sensors_event_t a, g, temp;

        if (xQueueReceive(sensorQueue, &receivedData, 0)) {
        temp.temperature = receivedData.temperature;
        a.acceleration.x = receivedData.accelX;
        a.acceleration.y = receivedData.accelY;
        a.acceleration.z = receivedData.accelZ;
        g.gyro.x = receivedData.gyroX;
        g.gyro.y = receivedData.gyroY;
        g.gyro.z = receivedData.gyroZ;
        }
        else {
            return lastFilteredAngle;
        }
        
        // Apply gyro offset
        gyroRate = - g.gyro.y - gyro_offset;
        
        // Calculate angle from accelerometer
        float accelAngle = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;
        
        // Apply angle offset (so 0 degrees = upright)
        accelAngle -= angle_offset;
        
        // Time delta for gyro integration
        unsigned long now = micros();
        float dt = (now - lastTimestamp) / 1000000.0;
        lastTimestamp = now;
        
        // Complementary filter (adjustable)
        float filterCoeff = 0.96;  // 96% gyro, 4% accelerometer
        filteredAngle = filterCoeff * (filteredAngle + gyroRate * dt) + (1.0 - filterCoeff) * accelAngle;
        
        lastFilteredAngle = filteredAngle;
        return filteredAngle;
    }

    float computePID(float currentAngle) {
        unsigned long now = micros();
        float dt = (now - lastLoopTime) / 1000000.0;
        lastLoopTime = now;
        
        // Calculate error
        float error = setpoint - currentAngle;
        
        // Integral with anti-windup (limit to smaller range)
        integral += error * dt;
        integral = constrain(integral, -1.0, 1.0);
        
        // Derivative (using actual gyro rate for smoother response)
        float derivative = -gyroRate;
        
        // Calculate output
        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        
        // Limit output to prevent saturation
        output = constrain(output, -0.99, 0.99);  // Reduced max speed
        
        return output;
    }

void taskBalancing(void *pvParameters) {

    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Reset PID
    integral = 0.0;
    previous_error = 0.0;
    filteredAngle = 0.0;
    lastLoopTime = micros();
    lastTimestamp = micros();
    
    // UDP setup
    udp.listen(8888);
    
    unsigned long lastPrintTime = 0;
    
    Serial.println("Balancing Active! Hold robot upright to test...\n");
    
    for(;;) {
        // Get angle
        float angle = getAngle();
        
        // Compute motor speed
        float motorSpeed = computePID(angle);
        
        // Apply to motors
        doOneStepOrNone(motorSpeed);
        
        // Print debug info
        unsigned long now = millis();
        
        if (now - lastPrintTime >= 200) {  // 20Hz update
            lastPrintTime = now;
            char buffer[256];            
            snprintf(buffer, 256, "Angle:%.3f, Gyro:%.3f, Motor:%.3f", angle, gyroRate, motorSpeed);
            Serial.printf("%s\n", buffer);
            udp.broadcast(buffer);
        }
        
    }
}
