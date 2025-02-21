#include <Wire.h>
#include <ESP32Servo.h>

// ----- MPU6050 Gyro -----
#define MPU6050_ADDR 0x68
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define GYRO_ZOUT_H 0x47
#define GYRO_SCALE 131.0  // Sensitivity factor for ±250°/s

// ----- PID Control (Improved for Fine Correction) -----
float kp = 0.014;  // Increase for stronger small error response
float ki = 0.0; // Increased to counter small steady-state errors
float kd = 0.024;  // Slightly reduced to avoid over-damping

float pid_error = 0, last_pid_error = 0, pid_integral = 0;
unsigned long prev_time;

// ----- Gyro Variables -----
float yaw = 0;
float gyro_z_offset = 0;
const float yaw_deadband = 0.5; // Small deadband to reduce noise impact

// ----- Steering Servo -----
#define SteeringServoPin 2
Servo steeringServo;
#define STEERING_CENTER 80  // Neutral position
#define STEERING_LEFT 120   // Max left
#define STEERING_RIGHT 30   // Max right

// ----- Motor Drive -----
#define PWMA 11
#define AIN1 7
#define AIN2 8
#define STBY 10
#define PWM_CHANNEL 1
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

void motor_driver_setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL);
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
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2, (bool)true); // Corrected function signature

  if (Wire.available() < 2) return 0; 
  int16_t rawZ = Wire.read() << 8 | Wire.read();

  static float filtered_gz = 0;
  filtered_gz = (0.9 * filtered_gz) + (0.1 * (rawZ / GYRO_SCALE)); // Moving average filter

  return filtered_gz - gyro_z_offset;
}

// MPU6050 initialization
void setupMPU6050() {
  writeMPU6050(PWR_MGMT_1, 0x00);
  delay(100);
  writeMPU6050(GYRO_CONFIG, 0x00);
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

void move_motor(int speed) {
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = abs(speed);
  }
  ledcWrite(PWM_CHANNEL, speed);
}

void stop_motor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CHANNEL, 0);
}

float calculateYawError(float targetYaw, float currentYaw) {
  float error = targetYaw - currentYaw;

  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  return (abs(error) < yaw_deadband) ? 0 : error; // Apply deadband to avoid small jitters
}

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

void loop() {
  float targetYaw = 0.0;

  move_motor(110);
  update_steering(targetYaw);
}
