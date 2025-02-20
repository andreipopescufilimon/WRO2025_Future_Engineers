#include <Wire.h>

#define MPU6050_ADDR 0x68
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define GYRO_ZOUT_H 0x47

#define GYRO_SCALE 131.0  // Sensitivity factor for ±250°/s

// Time tracking
unsigned long prev_time;
float yaw = 0;  // Yaw angle in degrees (0 to 360)

// Gyro offsets (calibration)
float gyro_z_offset = 0;

// Function to write to MPU6050 register
void writeMPU6050(byte reg, byte value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Function to read raw gyro Z-axis data
float readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDR, 2);

  int16_t rawZ = Wire.read() << 8 | Wire.read();
  return (rawZ / GYRO_SCALE) - gyro_z_offset;
}

// MPU6050 initialization
void setupMPU6050() {
  writeMPU6050(PWR_MGMT_1, 0x00); // Wake up MPU6050
  writeMPU6050(GYRO_CONFIG, 0x00); // Set gyro sensitivity to ±250°/s
}

// MPU6050 calibration to find offsets
void calibrateMPU6050() {
  int numSamples = 500;
  float sumZ = 0;
  
  for (int i = 0; i < numSamples; i++) {
    sumZ += readGyroZ();
    delay(3);
  }

  gyro_z_offset = sumZ / numSamples; // Calculate average offset
}

// Setup function
void setup() {
  Wire.begin();
  Serial.begin(115200);

  setupMPU6050();
  calibrateMPU6050();

  prev_time = millis();
}

// Loop function to calculate yaw angle
void loop() {
  float gz = readGyroZ();

  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0; // Time in seconds
  prev_time = current_time;

  // Adjust rotation to match the clockwise direction
  yaw -= gz * dt; // Subtract instead of adding for clock-wise rotation

  // Keep yaw between 0 - 360 degrees
  if (yaw >= 360) yaw -= 360;
  if (yaw < 0) yaw += 360;

  // Print yaw angle
  Serial.print("Yaw (Clockwise): ");
  Serial.print(yaw);
  Serial.println("°");

  delay(10);
}
