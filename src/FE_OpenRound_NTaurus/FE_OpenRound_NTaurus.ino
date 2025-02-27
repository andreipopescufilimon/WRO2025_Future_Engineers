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
#define STEERING_LEFT 120   // Max left
#define STEERING_CENTER 80  // Neutral position
#define STEERING_RIGHT 30   // Max right

// ----- UART Communication -----
#define RX_PIN 0
#define TX_PIN 1
SoftwareSerial cameraSerial(RX_PIN, TX_PIN);
String receivedMessage = "";
char command = '0';

// ----- Turns count and direction -----
char turn_direction = '0';  // Turns direction based on first viewed orange or blue line
int turn_count = 0;         // Counter for turns
const int max_turns = 12;   // Maximum turns before stopping

// ----- Motor Drive -----
#define PWMA 9   // PWM pin for motor speed control
#define AIN1 7   // Motor direction pin 1
#define AIN2 8   // Motor direction pin 2
#define STBY 10  // Standby pin

#define PWM_CHANNEL 5     // ESP32 PWM channel (0-15)
#define PWM_FREQ 1000     // PWM frequency in Hz
#define PWM_RESOLUTION 8  // PWM resolution (8-bit: 0-255)

// ----- Speed Control -----
int robot_speed = 120;
int turn_speed = 110;

// ----- Gyro Variables -----
float yaw = 0;
float targetYaw = 0;
float gyro_z_offset = 0;
unsigned long prev_time, last_pid_time = 0;
const int PID_INTERVAL = 10;

// ----- PID Control for straight movement -----
float kp = 0.14;
float ki = 0.015;
float kd = 0.24;

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
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true);
  if (Wire.available() < 2) return 0.0;
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


//////////////////////////////////////////////////////////
//                Motor Functions                       //
//////////////////////////////////////////////////////////

// Motor driver setup
void motor_driver_setup() {
  // Initialize motor control pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable the motor driver by setting STBY (standby) pin HIGH
  digitalWrite(STBY, HIGH);

  // Configure LEDC PWM for motor speed control
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL);
}

// Move motor at specified speed
void move(int speed) {
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;  // Convert negative speed to positive
  }

  ledcWrite(PWM_CHANNEL, speed);  // Set motor speed using LEDC PWM
}

// Stop motor
void stop_motor() {
  move(-10);
  custom_delay(20);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, 0);
}

// Function to turn X degrees (direction: 'l' / 'L' for left, 'r' / 'R' for right)
void turn(char direction, int degrees) {
  // Determine target yaw based on direction.
  if (direction == 'L' || direction == 'l') {
    targetYaw = yaw - degrees;
  } else if (direction == 'R' || direction == 'r') {
    targetYaw = yaw + degrees;
  } else {
    Serial.println("Invalid direction. Use 'L' or 'R'.");
    return;
  }

  // Normalize targetYaw to be within 0-359 degrees.
  if (targetYaw >= 360) targetYaw -= 360;
  if (targetYaw < 0) targetYaw += 360;

  // Debug print.
  if (debug) {
    Serial.print("Turning ");
    Serial.print((direction == 'L' || direction == 'l') ? "LEFT" : "RIGHT");
    Serial.print(" ");
    Serial.print(degrees);
    Serial.println(" degrees...");
  }

  // Set the steering angle based on the direction.
  if (direction == 'L' || direction == 'l') {
    steeringServo.write(STEERING_LEFT);
  } else {  // direction is 'R' or 'r'
    steeringServo.write(STEERING_RIGHT);
  }

  // Begin the turn by moving the robot.
  move(turn_speed);

  // Keep turning until within 10 degrees of the target yaw.
  while (abs(targetYaw - yaw) > 10) {
    float gz = readGyroZ();
    unsigned long current_time = millis();
    float dt = (current_time - prev_time) / 1000.0;
    prev_time = current_time;

    // Update yaw based on gyro data.
    yaw -= gz * dt;

    // Normalize yaw to be within 0-359 degrees.
    if (yaw >= 360) yaw -= 360;
    if (yaw < 0) yaw += 360;

    // Continue moving at the set robot speed.
    move(turn_speed);
  }

  // Center the steering after completing the turn.
  steeringServo.write(STEERING_CENTER);

  if (debug) {
    Serial.println("Turn complete. Centering steering.");
  }
}

//////////////////////////////////////////////////////////
//              Steering Functions                      //
//////////////////////////////////////////////////////////

// Steering servo setup
void steering_servo_setup() {
  // Initialize servo control pins
  steeringServo.attach(SteeringServoPin);

  // Test servo positions
  steeringServo.write(STEERING_LEFT);
  custom_delay(500);
  steeringServo.write(STEERING_CENTER);
  custom_delay(500);
  steeringServo.write(STEERING_RIGHT);
  custom_delay(500);
  steeringServo.write(STEERING_CENTER);
  custom_delay(500);
}

// ----- Move steerign to angle -----
void steer(int steering_angle) {
  steeringServo.write(steering_angle);
}

// ----- PID Steering Correction -----
void update_steering_move(float targetYaw) {
    unsigned long current_time = millis();
    if (current_time - last_pid_time < PID_INTERVAL) return;
    last_pid_time = current_time;
    
    float dt = (current_time - prev_time) / 1000.0;
    prev_time = current_time;
    if (dt > 0.1) dt = 0.1;
    
    float gz = readGyroZ();
    yaw -= gz * dt;
    
    if (yaw >= 360) yaw -= 360;
    if (yaw < 0) yaw += 360;
    
    pid_error = calculateYawError(targetYaw, yaw);
    pid_integral += pid_error * dt;
    float pid_derivative = (pid_error - last_pid_error) / dt;
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
  while (millis() - start_time < delay_time) {
  }
}

void execute_command(char command) {
  if (command == 'B') {
    if (debug) {
      Serial.println("Received 'Black' command. Turning...");
    }
    turn(turn_direction, 90);
    turn_count++;
    command = 'X';
  } else {
    update_steering_move(targetYaw);
    move(robot_speed);
  }
}


//////////////////////////////////////////////////////////
//                      Main Code                       //
//////////////////////////////////////////////////////////

// ----- Main Setup -----
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
