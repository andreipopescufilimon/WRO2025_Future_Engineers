#include <Wire.h>

// ----- MPU6050 Gyro -----
#define MPU6050_ADDR 0x68
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define GYRO_ZOUT_H 0x47
#define GYRO_SCALE 131.0  // Sensitivity factor for ±250°/s

float yaw = 0;
float gyro_z_offset = 0;
unsigned long prev_time;

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
  writeMPU6050(PWR_MGMT_1, 0x00);
  writeMPU6050(GYRO_CONFIG, 0x00);
}

// MPU6050 calibration
void calibrateMPU6050() {
  int numSamples = 500;
  float sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    sumZ += readGyroZ();
    delay(3);
  }

  gyro_z_offset = sumZ / numSamples;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  setupMPU6050();
  calibrateMPU6050();
  prev_time = millis();

  Serial.println("MPU6050 Initialized and Calibrated");
}

void loop() {
  float gz = readGyroZ();
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;
  prev_time = current_time;

  yaw -= gz * dt;

  if (yaw >= 360) yaw -= 360;
  if (yaw < 0) yaw += 360;

  Serial.print("Yaw: ");
  Serial.println(yaw);
  
  delay(50);  // Small delay for better readability
}
