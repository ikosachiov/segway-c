#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// The MPU 6050 gyroscope is positioned so that the X-axis is directed backward,
// the Y-axis is directed to the right, and the Z-axis is directed upward.
// The positive direction of rotation along the axes is clockwise.


// Add this ONE LINE after your includes
Adafruit_MPU6050 mpu;

// Calibration variables
float angle_offset = 0.0;
float gyro_offset = 0.0;
bool calibrated = false;

// Queue handle
QueueHandle_t sensorQueue;
const int QUEUE_SIZE = 10;  // Store up to 10 readings

// Structure to hold all sensor data
typedef struct {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float temperature;
  unsigned long timestamp;
} SensorData;

void setupAccel() {
    calibrateMPU();
    sensorQueue = xQueueCreate(QUEUE_SIZE, sizeof(SensorData));
}

void calibrateMPU() {
    Serial.println("=== MPU6050 CALIBRATION ===");
    Serial.println("Place robot on a LEVEL surface!");
    Serial.println("Calibrating in 3 seconds...");

    Wire.begin(21, 22);
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 Ready!");
    
    // Calibrate
   
    float sum_angle = 0.0;
    float sum_gyro = 0.0;
    const int samples = 200;
    
    for (int i = 0; i < samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        // Calculate angle from accelerometer
        float angle = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;
        sum_angle += angle;
        sum_gyro += g.gyro.y;
        
        delay(10);
        
        if (i % 20 == 0) Serial.print(".");
    }
    
    angle_offset = sum_angle / samples;
    gyro_offset = sum_gyro / samples;
    calibrated = true;
    
    Serial.println("\nCalibration complete!");
    Serial.printf("Angle offset: %.2f degrees\n", angle_offset);
    Serial.printf("Gyro offset: %.3f deg/s\n", gyro_offset);
}

// Sensor reading task on Core 0
void taskAccel(void *parameter) {
  sensors_event_t a, g, temp;
  SensorData data;
  
  for (;;) {
    // Read sensor
    mpu.getEvent(&a, &g, &temp);
    
    // Pack data
    data.accelX = a.acceleration.x;
    data.accelY = a.acceleration.y;
    data.accelZ = a.acceleration.z;
    data.gyroX = g.gyro.x;
    data.gyroY = g.gyro.y;
    data.gyroZ = g.gyro.z;
    data.temperature = temp.temperature;
    data.timestamp = millis();
    
    // Send to queue (non-blocking, discard if queue is full)
    xQueueSend(sensorQueue, &data, 0);
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

