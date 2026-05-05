#include <Wire.h>

// MPU6050 Registers
#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1   0x6B
#define CONFIG       0x1A

// Calibration and State
double angle_offset = 0.0;
double gyro_offset = 0.0;
bool calibrated = false;

QueueHandle_t sensorQueue;
const int QUEUE_SIZE = 1; // Only the freshest data matters

typedef struct {
  double accelX, accelZ; // Only X and Z needed for 2D balance
  double gyroY;          // Only Y needed for 2D balance
  unsigned long timestamp;
} SensorData;

void setupAccel() {
    Wire.begin(21, 22);
    Wire.setClock(400000); // ⚡ 400kHz Fast Mode
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0); 
    Wire.endTransmission(true);

    // Set Filter Bandwidth to 260Hz (Zero Latency)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(CONFIG);
    Wire.write(0); 
    Wire.endTransmission(true);

    calibrateMPU();
    sensorQueue = xQueueCreate(QUEUE_SIZE, sizeof(SensorData));
}

void calibrateMPU() {
    Serial.println("=== MPU6050 RAW CALIBRATION ===");
    const int samples = 500;
    double sum_angle = 0.0;
    double sum_gyro = 0.0;

    for (int i = 0; i < samples; i++) {
        int16_t ax, az, gy;
        
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 14, true);

        ax = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read(); // Skip Accel Y
        az = Wire.read() << 8 | Wire.read();
        Wire.read(); Wire.read(); // Skip Temp
        Wire.read(); Wire.read(); // Skip Gyro X
        gy = Wire.read() << 8 | Wire.read();

        // Convert to degrees and deg/s (Default scales: 16384 LSB/g, 131 LSB/deg/s)
        sum_angle += atan2(ax / 16384.0, az / 16384.0) * 180.0 / PI;
        sum_gyro += gy / 131.0;
        
        if (i % 50 == 0) Serial.print(".");
        delay(2);
    }

    angle_offset = sum_angle / samples;
    gyro_offset = sum_gyro / samples;
    calibrated = true;

    Serial.println("\nCalibration complete!");
    Serial.printf("Offset: Angle: %.2f, Gyro: %.3f\n", angle_offset, gyro_offset);
}

void taskAccel(void *parameter) {
  SensorData data;
  int16_t ax, az, gy;

  for (;;) {
    // ⚡ Fast Burst Read (14 bytes in one transaction)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); 
    az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); 
    Wire.read(); Wire.read(); 
    gy = Wire.read() << 8 | Wire.read();

    data.accelX = ax / 16384.0;
    data.accelZ = az / 16384.0;
    data.gyroY = gy / 131.0;
    data.timestamp = micros(); // ⚡ Use micros for PID dt precision

    // ⚡ Use Overwrite: PID task always gets the newest frame
    xQueueOverwrite(sensorQueue, &data);

    // Shortest possible delay to yield to other tasks
    delayMicroseconds(500); 
  }
}

