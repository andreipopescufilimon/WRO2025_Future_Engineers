#include <Wire.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>

// ----- MPU6050 Gyro -----
#define MPU6050_ADDR 0x68
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B
#define GYRO_ZOUT_H 0x47
#define GYRO_SCALE 131.0  // Sensitivity factor for ±250°/s

// ----- Steering Servo -----
#define SteeringServoPin 2
Servo steeringServo;
#define STEERING_LEFT 110   // Max left
#define STEERING_CENTER 79  // Neutral position
#define STEERING_RIGHT 40   // Max right

// ----- Turn Offset Adjustments -----
// Right-turn extra offset (added when turning right)
#define RIGHT_TURN_OFFSET 5  // Base extra degrees for right turns
#define MIN_RIGHT_OFFSET 2   // Minimum extra degrees for right turns

// Left-turn extra offset (added when turning left to compensate understeer)
#define LEFT_TURN_OFFSET 5   // Base extra degrees for left turns
#define EXTRA_LEFT_OFFSET 1

// ----- UART Communication -----
#define RX_PIN 0
#define TX_PIN 1
SoftwareSerial cameraSerial(RX_PIN, TX_PIN);
String receivedMessage = "";
char command = '0';

// ----- Turns count and direction -----
char turn_direction = '0';  // Turns direction based on first viewed orange or blue line
int turn_count = 0;         // Counter for turns
const int max_turns = 12;   // Maximum turns before stopping (3 laps for a square track)
unsigned long lastTurnTime = 0;
const unsigned long turnCooldown = 1000;

// ----- Motor Drive -----
#define PWMA 9    // PWM pin for motor speed control
#define AIN1 7    // Motor direction pin 1
#define AIN2 8    // Motor direction pin 2
#define STBY 10   // Standby pin

#define PWM_CHANNEL 5      // ESP32 PWM channel (0-15)
#define PWM_FREQ 1000      // PWM frequency in Hz
#define PWM_RESOLUTION 8   // PWM resolution (8-bit: 0-255)

// ----- Speed Control -----
int robot_speed = 100;
int turn_speed = 90;

// ----- Gyro Variables -----
float yaw = 0;
float targetYaw = 0;
float gyro_z_offset = 0;
unsigned long prev_time, last_pid_time = 0;
const int PID_INTERVAL = 1;

// ----- PID Control for straight movement -----
float kp = 4.5;
float ki = 0.0;
float kd = 8.3;
float pid_error = 0, last_pid_error = 0, pid_integral = 0;

// ----- Debug Mode -----
bool debug = false;  // Set to true for serial debugging

//////////////////////////////////////////////////////////
//                Gyro Functions                        //
//////////////////////////////////////////////////////////

void setupMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();
}

void calibrateMPU6050() {
  int numSamples = 1000;
  float sumZ = 0;
  for (int i = 0; i < numSamples; i++) {
    sumZ += readGyroZ();
    delay(3);
  }
  gyro_z_offset = sumZ / numSamples;
}

void writeMPU6050(byte reg, byte value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

float readRawGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  // Cast parameters as needed for ESP32 Wire library
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)2, (bool)true);
  int16_t rawZ = Wire.read() << 8 | Wire.read();
  return rawZ / GYRO_SCALE;
}

float readGyroZ() {
  return readRawGyroZ() - gyro_z_offset;
}

float calculateYawError(float targetYaw, float currentYaw) {
  float error = targetYaw - currentYaw;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  return error;
}

//////////////////////////////////////////////////////////
//                Motor Functions                       //
//////////////////////////////////////////////////////////

void motor_driver_setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL);
}

void move(int speed) {
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;
  }
  ledcWrite(PWM_CHANNEL, speed);
}

void stop_motor() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, 0);
}

//////////////////////////////////////////////////////////
//                     Turn Function                    //
//////////////////////////////////////////////////////////

// Turn X degrees; direction: 'L' for left, 'R' for right
void turn(char direction, int degrees) {
  float initialYaw = yaw;
  float dynamic_offset = 0;  // Declare dynamic_offset here
  if (direction == 'L' || direction == 'l') {
    // For left turns, add extra offset to compensate understeer.
    // Here we always add a fixed extra offset to turn further left.
    dynamic_offset = LEFT_TURN_OFFSET + EXTRA_LEFT_OFFSET;
    targetYaw = initialYaw - (degrees + dynamic_offset);
  } else if (direction == 'R' || direction == 'r') {
    // For right turns, use the previous dynamic offset calculation.
    if(turn_count >= (max_turns - 4)) {
      dynamic_offset = RIGHT_TURN_OFFSET + 1;  // Extra steering on the last lap
    } else {
      dynamic_offset = RIGHT_TURN_OFFSET - (turn_count * 0.5);
      if (dynamic_offset < MIN_RIGHT_OFFSET) dynamic_offset = MIN_RIGHT_OFFSET;
    }
    targetYaw = initialYaw + degrees + dynamic_offset;
  } else {
    Serial.println("Invalid direction. Use 'L' or 'R'.");
    return;
  }

  // Normalize targetYaw within 0-359 degrees.
  if (targetYaw >= 360) targetYaw -= 360;
  if (targetYaw < 0) targetYaw += 360;

  if (debug) {
    Serial.print("Turning ");
    Serial.print((direction == 'L' || direction == 'l') ? "LEFT" : "RIGHT");
    Serial.print(" by ");
    Serial.print(degrees);
    if (direction == 'L' || direction == 'l') {
      Serial.print(" + dynamic offset(");
      Serial.print(dynamic_offset);
      Serial.print(") [Left]");
    } else {
      Serial.print(" + dynamic offset(");
      Serial.print(dynamic_offset);
      Serial.print(") [Right]");
    }
    Serial.println(" degrees...");
  }

  // Set steering for turn.
  if (direction == 'L' || direction == 'l') {
    steeringServo.write(STEERING_LEFT);
  } else {
    steeringServo.write(STEERING_RIGHT);
  }

  move(turn_speed);
  const float turnThreshold = 5.0;  // Acceptable error in degrees.
  while (abs(calculateYawError(targetYaw, yaw)) > turnThreshold) {
    float gz = readGyroZ();
    unsigned long current_time = millis();
    float dt = (current_time - prev_time) / 1000.0;
    prev_time = current_time;
    yaw -= gz * dt;
    if (yaw >= 360) yaw -= 360;
    if (yaw < 0) yaw += 360;
    move(turn_speed);
  }
  // Reset yaw and PID terms after turn.
  yaw = targetYaw;
  pid_integral = 0;
  last_pid_error = 0;
  steeringServo.write(STEERING_CENTER);
  if (debug) {
    Serial.println("Turn complete. Steering centered and yaw reset.");
  }
}

//////////////////////////////////////////////////////////
//              Steering Functions                      //
//////////////////////////////////////////////////////////

void steering_servo_setup() {
  steeringServo.attach(SteeringServoPin);
  steeringServo.write(STEERING_CENTER);
  custom_delay(500);
}

void steer(int steering_angle) {
  steeringServo.write(steering_angle);
}

void update_steering_move(float targetYaw) {
  unsigned long current_time = millis();
  if (current_time - last_pid_time < PID_INTERVAL) return;
  float dt = (current_time - prev_time) / 1000.0;
  prev_time = current_time;
  if (dt > 0.1) dt = 0.1;
  last_pid_time = current_time;
  
  float gz = readGyroZ();
  yaw -= gz * dt;
  if (yaw >= 360) yaw -= 360;
  if (yaw < 0) yaw += 360;
  
  // Dynamic gyro offset adjustment.
  float raw_gz = readRawGyroZ();
  float measured_gz = raw_gz - gyro_z_offset;
  const float driftThreshold = 0.5; // degrees/second threshold
  const float adjustmentFactor = 0.005; // adjustment factor
  if (abs(measured_gz) < driftThreshold) {
    gyro_z_offset += adjustmentFactor * measured_gz;
  }
  
  pid_error = calculateYawError(targetYaw, yaw);
  pid_integral += (pid_error * dt);
  float pid_derivative = (pid_error - last_pid_error);
  last_pid_error = pid_error;
  
  float correction = (kp * pid_error) + (ki * pid_integral) + (kd * pid_derivative);
  int steering_angle = STEERING_CENTER - correction;
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

//////////////////////////////////////////////////////////
//                Custom Functions                      //
//////////////////////////////////////////////////////////

void custom_delay(long long delay_time) {
  long long start_time = millis();
  while (millis() - start_time < delay_time) { }
}

void execute_command(char command) {
  if (command == 'B') {
    if (millis() - lastTurnTime < turnCooldown) {
      if (debug) Serial.println("Ignoring repeated 'B' command due to cooldown.");
      return;
    }
    if (debug) Serial.println("Received 'Black' command. Turning...");
    turn(turn_direction, 90);
    turn_count++;
    lastTurnTime = millis();
    command = '0';
    pid_integral = 0;
  } else {
    update_steering_move(targetYaw);
    move(robot_speed);
  }
}

//////////////////////////////////////////////////////////
//                      Main Code                       //
//////////////////////////////////////////////////////////

void setup() {
  Wire.begin();
  Serial.begin(115200);
  cameraSerial.begin(19200);
  steering_servo_setup();
  motor_driver_setup();
  setupMPU6050();
  calibrateMPU6050();
  prev_time = millis();
}

void loop() {
  // After finishing the third lap (max_turns reached), move for 2 more seconds
  if (turn_count >= max_turns) {
    unsigned long t = millis();
    while (millis() - t < 500) {
      update_steering_move(targetYaw);
      move(robot_speed);
    }
    stop_motor();
    if (debug) Serial.println("Maximum turns reached. Stopping...");
    while (true);
  }
  
  command = '0';
  while (cameraSerial.available() > 0) {
    char receivedChar = cameraSerial.read();
    if (receivedChar == '\n') {
      if (debug) {
        Serial.print("Received from OpenMV: ");
        Serial.println(receivedMessage);
      }
      receivedMessage.toUpperCase();
      if (receivedMessage.indexOf("BLACK") != -1) {
        command = 'B';
      } else if (receivedMessage.indexOf("BLUE") != -1) {
        command = 'L';
        if (turn_direction == '0') turn_direction = 'L';
      } else if (receivedMessage.indexOf("ORANGE") != -1) {
        command = 'O';
        if (turn_direction == '0') turn_direction = 'R';
      } else if (receivedMessage.indexOf("RED") != -1) {
        command = 'R';
      } else if (receivedMessage.indexOf("GREEN") != -1) {
        command = 'G';
      } else if (receivedMessage.indexOf("PINK") != -1) {
        command = 'P';
      }
      execute_command(command);
      if (debug) {
        Serial.print("Current command: ");
        Serial.println(command);
      }
      receivedMessage = "";
    } else {
      receivedMessage += receivedChar;
    }
  }
  update_steering_move(targetYaw);
  move(robot_speed);
}
