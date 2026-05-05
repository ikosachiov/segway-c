// Add these includes at the VERY TOP of your file (before anything else)
#include <Wire.h>
#include <AsyncUDP.h>

AsyncUDP udp;


#define STEP0 25
#define DIR0  33
#define STEP1 26
#define DIR1  27

unsigned long lastStepTime = 0;

const double MAX_STEPS_PER_SEC = 2500.0;
const double MIN_STEP_INTERVAL_US = 1000000.0 / MAX_STEPS_PER_SEC;

// REDUCED PID constants for less aggressive balancing
double Kp = 0.08;
double Ki = 0.00;
double Kd = 0.18;

// PID variables
double setpoint = 0.0;
double integral = 0.0;
double previous_error = 0.0;
unsigned long lastLoopTime = 0;

//
double lastFilteredAngle = 0.0;

// Complementary filter
double filteredAngle = 0.0;
double gyroRate = 0.0;
unsigned long lastTimestamp = 0;

void setupBalancing() {
    pinMode(STEP0, OUTPUT);
    pinMode(DIR0, OUTPUT);
    pinMode(STEP1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    
    digitalWrite(STEP0, LOW);
    digitalWrite(STEP1, LOW);
}

void doOneStepOrNone(double speed) {
    if (fabs(speed) < 0.04) return;  // Dead zone to prevent jitter
    
    // Set direction
    
    digitalWrite(DIR1, speed > 0 ? HIGH : LOW);
    digitalWrite(DIR0, speed > 0 ? HIGH : LOW);    
    
    // Speed 0.95 = ~2375 steps/sec = 420us interval
    // Speed 0.05 = 125 steps/sec = 8000us interval
    unsigned long stepIntervalUs = (unsigned long)(MIN_STEP_INTERVAL_US / abs(speed));
    
    unsigned long now = micros();

    if (now - lastStepTime >= stepIntervalUs) {
        digitalWrite(STEP1, HIGH);        
        digitalWrite(STEP0, HIGH);        
        delayMicroseconds(10);
        digitalWrite(STEP1, LOW);                
        digitalWrite(STEP0, LOW);
        delayMicroseconds(10);
        lastStepTime = now;
    }
}

double getAngle() {
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
    double accelAngle = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;
    
    // Apply angle offset (so 0 degrees = upright)
    accelAngle -= angle_offset;
    
    // Time delta for gyro integration
    unsigned long now = micros();
    double dt = (now - lastTimestamp) / 1000000.0;
    lastTimestamp = now;
    
    // Complementary filter (adjustable)
    double filterCoeff = 0.96;  // 96% gyro, 4% accelerometer
    filteredAngle = filterCoeff * (filteredAngle + gyroRate * dt) + (1.0 - filterCoeff) * accelAngle;
    
    lastFilteredAngle = filteredAngle;
    return filteredAngle;
}

double computePID(double currentAngle) {
    unsigned long now = micros();
    double dt = (now - lastLoopTime) / 1000000.0;
    lastLoopTime = now;
    
    // Calculate error
    double error = setpoint - currentAngle;
    
    // Integral with anti-windup (limit to smaller range)
    integral += error * dt;
    integral = constrain(integral, -1.0, 1.0);
    
    // Calculate output
    // double output = (Kp * error) + (Ki * integral) + Kd * ((error - previous_error) / dt);
    double output = (Kp * error) + (Ki * integral) + Kd * (-gyroRate);

    if (millis() % 100 == 0) { 
        char buffer[256];            
        // snprintf(buffer, 256, "Kp*e:%.4f, Ki*i:%.4f, Kd*de/dt:%.8f", (Kp * error), (Ki * integral), Kd * ((error - previous_error) / dt));
        snprintf(buffer, 256, "Kp*e:%.4f, Ki*i:%.4f, Kd*(-gR):%.8f", (Kp * error), (Ki * integral), Kd * (-gyroRate));
        Serial.printf("%s\n", buffer);
        udp.broadcast(buffer);
    }
    
    // Limit output to prevent saturation
    output = constrain(output, -0.99, 0.99);  // Reduced max speed
    
    previous_error = error;
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
      
    Serial.println("Balancing Active! Hold robot upright to test...\n");
    
    for(;;) {
        // Get angle
        double angle = getAngle();
        
        // Compute motor speed
        double motorSpeed = computePID(angle);
        
        // Apply to motors
        doOneStepOrNone(motorSpeed);
        
        // Print debug info
        
        if (millis() % 100 == 0) {
            char buffer[256];            
            snprintf(buffer, 256, "Angle:%.4f, Gyro:%.4f, Motor:%.4f", angle, gyroRate, motorSpeed);
            Serial.printf("%s\n", buffer);
            udp.broadcast(buffer);
        }
        
    }
}
