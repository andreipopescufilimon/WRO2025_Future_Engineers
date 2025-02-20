#include <Wire.h>
#include <Servo.h>

// ----- MPU6050 Gyro -----
#define MPU6050_ADDR 0x68
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define GYRO_ZOUT_H 0x47
#define GYRO_SCALE 131.0  // Sensitivity factor for ±250°/s

// ----- PID Control (Improved for Fine Correction) -----
float kp = 0.14;  // Increase for stronger small error response
float ki = 0.015; // Increased to counter small steady-state errors
float kd = 0.24;  // Slightly reduced to avoid over-damping

float pid_error = 0, last_pid_error = 0, pid_integral = 0;
unsigned long prev_time;

// ----- Gyro Variables -----
float yaw = 0;
float gyro_z_offset = 0;

// ----- Steering Servo -----
#define SteeringServoPin 3
Servo steeringServo;
#define STEERING_CENTER 80  // Neutral position
#define STEERING_LEFT 120   // Max left
#define STEERING_RIGHT 30   // Max right

// ----- Motor Drive -----
#define PWMA 11
#define AIN1 7
#define AIN2 8
#define STBY 10

void motor_driver_setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

// Function to write to MPU6050 register
void writeMPU6050(byte reg, byte value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Function to read raw gyro Z-axis data with filtering
float readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);

  if (Wire.available() < 2) return 0; 
  int16_t rawZ = Wire.read() << 8 | Wire.read();

  // Apply a simple low-pass filter for noise reduction
  static float filtered_gz = 0;
  filtered_gz = (0.8 * filtered_gz) + (0.2 * (rawZ / GYRO_SCALE));

  return filtered_gz - gyro_z_offset;
}

// MPU6050 initialization
void setupMPU6050() {
  writeMPU6050(PWR_MGMT_1, 0x00); // Wake up MPU6050
  delay(100);
  writeMPU6050(GYRO_CONFIG, 0x00); // Set gyro sensitivity to ±250°/s
}

// MPU6050 calibration to find offsets
void calibrateMPU6050() {
  int numSamples = 500;
  float sumZ = 0;

  Serial.print("Calibrating MPU6050...");
  for (int i = 0; i < numSamples; i++) {
    sumZ += readGyroZ();
    delay(3);
  }

  gyro_z_offset = sumZ / numSamples;
  Serial.println(" Done!");
}

// ----- Motor Functions -----
void move_motor(int speed) {
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = abs(speed);
  }
  analogWrite(PWMA, speed);
}

void stop_motor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}

// ----- Yaw Error Calculation with Angle Wrapping -----
float calculateYawError(float targetYaw, float currentYaw) {
  float error = targetYaw - currentYaw;

  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  return error;
}

// ----- PID Steering Correction -----
void update_steering(float targetYaw) {
  float gz = readGyroZ();

  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0; 
  prev_time = current_time;

  if (dt > 0.1) dt = 0.1; 

  yaw -= gz * dt; 

  if (yaw >= 360) yaw -= 360;
  if (yaw < 0) yaw += 360;

  pid_error = calculateYawError(targetYaw, yaw);
  pid_integral += pid_error * dt;
  float pid_derivative = (pid_error - last_pid_error) / dt;
  last_pid_error = pid_error;

  float correction = (kp * pid_error) + (ki * pid_integral) + (kd * pid_derivative);

  // **Scaled correction for finer adjustments**
  int steering_angle = STEERING_CENTER - (correction);
  steering_angle = constrain(steering_angle, STEERING_RIGHT, STEERING_LEFT);

  steeringServo.write(steering_angle);

  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(" | Target: ");
  Serial.print(targetYaw);
  Serial.print(" | Error: ");
  Serial.print(pid_error);
  Serial.print(" | Correction: ");
  Serial.print(correction);
  Serial.print(" | Steering: ");
  Serial.println(steering_angle);
}

// ----- Main Setup -----
void setup() {
  Wire.begin();
  Serial.begin(115200);

  delay(1000);
  setupMPU6050();
  calibrateMPU6050();
  
  prev_time = millis();

  motor_driver_setup();
  steeringServo.attach(SteeringServoPin);
  steeringServo.write(STEERING_LEFT);
  delay(500);
  steeringServo.write(STEERING_CENTER);
  delay(500);
  steeringServo.write(STEERING_RIGHT);
  delay(500);
  steeringServo.write(STEERING_CENTER);
}

// ----- Main Loop -----
void loop() {
  float targetYaw = 0.0;

  move_motor(110);
  update_steering(targetYaw);
}
