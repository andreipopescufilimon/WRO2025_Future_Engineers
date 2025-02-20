#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ----- MPU6050 Gyro -----
#define MPU6050_ADDR 0x68
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define GYRO_ZOUT_H 0x47
#define GYRO_SCALE 131.0  // Sensitivity factor for ±250°/s

// ----- Steering Servo -----
#define SteeringServoPin 3
Servo steeringServo;
#define STEERING_CENTER 80  // Neutral position
#define STEERING_LEFT 120   // Max left
#define STEERING_RIGHT 30   // Max right

// ----- UART Communication -----
#define RX_PIN 0
#define TX_PIN 1
SoftwareSerial cameraSerial(RX_PIN, TX_PIN);
String receivedMessage = "";
String command = "0";
int turn_direction = 0;
int turn_count = 0;        // Counter for turns
const int max_turns = 12;  // Maximum turns before stopping

// ----- Motor Drive -----
#define PWMA 11
#define AIN1 7
#define AIN2 8
#define STBY 10

// ----- Gyro Variables -----
float yaw = 0;
float targetYaw = 0;
float gyro_z_offset = 0;
unsigned long prev_time;

// ----- PID Control -----
float kp = 0.14;
float ki = 0.015;
float kd = 0.24;

float pid_error = 0, last_pid_error = 0, pid_integral = 0;

// ----- Debug Mode -----
bool debug = false;  // Set to true for serial debugging


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

// ----- Yaw Error Calculation with Angle Wrapping -----
float calculateYawError(float targetYaw, float currentYaw) {
  float error = targetYaw - currentYaw;

  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  return error;
}

// Function to turn X degrees (1 - Right; -1 - Left)
void turn(int direction, int degrees) {
  targetYaw = yaw + (direction == 1 ? -degrees : degrees);
  if (targetYaw >= 360) targetYaw -= 360;
  if (targetYaw < 0) targetYaw += 360;

  if (debug) {
    Serial.print("Turning ");
    Serial.print(direction == 1 ? "RIGHT" : "LEFT");
    Serial.print(" ");
    Serial.print(degrees);
    Serial.println(" degrees...");
  }

  steeringServo.write(direction == 1 ? STEERING_RIGHT : STEERING_LEFT);
  move(110);

  while (abs(targetYaw - yaw) > 3) {
    float gz = readGyroZ();
    unsigned long current_time = millis();
    float dt = (current_time - prev_time) / 1000.0;
    prev_time = current_time;
    yaw -= gz * dt;

    if (yaw >= 360) yaw -= 360;
    if (yaw < 0) yaw += 360;
  }

  steeringServo.write(STEERING_CENTER);
  //stop_motor();
  if (debug) {
    Serial.println("Turn complete. Centering steering.");
  }
}

// Motor driver setup
void motor_driver_setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

// Move motor at specified speed
void move(int speed) {
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

// Stop motor
void stop_motor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
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

  if (debug) {
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
}

// ----- Main Setup -----
void setup() {
  Wire.begin();
  Serial.begin(115200);
  cameraSerial.begin(19200);

  setupMPU6050();
  calibrateMPU6050();
  prev_time = millis();

  steeringServo.attach(SteeringServoPin);
  steeringServo.write(STEERING_CENTER);
  motor_driver_setup();
}

// ----- Main Loop -----
void loop() {
  if (turn_count >= max_turns) {
    stop_motor();
    if (debug) {
      Serial.println("Maximum turns reached. Stopping...");
    }
    while (true)
      ;
  }

  command = "0";
  while (cameraSerial.available() > 0) {
    char receivedChar = cameraSerial.read();

    if (receivedChar == '\n') {
      if (debug) {
        Serial.print("Received from OpenMV: ");
        Serial.println(receivedMessage);
      }

      if (receivedMessage.indexOf("BLACK") != -1) {
        command = "BLACK";
      } else if (receivedMessage.indexOf("BLUE") != -1) {
        command = "BLUE";
        if (turn_direction == 0) turn_direction = -1;
      } else if (receivedMessage.indexOf("ORANGE") != -1) {
        command = "ORANGE";
        if (turn_direction == 0) turn_direction = 1;
      } else if (receivedMessage.indexOf("RED") != -1) {
        command = "RED";
      } else if (receivedMessage.indexOf("GREEN") != -1) {
        command = "GREEN";
      } else if (receivedMessage.indexOf("PINK") != -1) {
        command = "PINK";
      }
      receivedMessage = "";
    } else {
      receivedMessage += receivedChar;
    }
  }

  if (debug) {
    Serial.print("Current command: ");
    Serial.println(command);
  }

  if (command != "0") {
    if (command == "BLACK") {
      if (debug) {
        Serial.println("Received 'Black' command. Turning...");
      }
      turn(turn_direction, 90);
      turn_count++;
      command = "X";
    } else {
      update_steering(targetYaw);
      move(110);
    }
  }
}
