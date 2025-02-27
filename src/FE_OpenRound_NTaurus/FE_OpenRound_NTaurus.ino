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
#define PWMA 9  // PWM pin for motor speed control
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
unsigned long prev_time;

// ----- PID Control for straight movement -----
float kp = 0.14;
float ki = 0.015;
float kd = 0.24;
float pid_error = 0, last_pid_error = 0, pid_integral = 0;

// ----- Debug Mode -----
bool debug = true;  // Set to true for serial debugging

// ----- Global variable for gyro reading from task -----
volatile float gyroZValue = 0;



//////////////////////////////////////////////////////////
//                Gyro Functions                        //
//////////////////////////////////////////////////////////

// Initialize MPU6050
void setupMPU6050() {
  writeMPU6050(PWR_MGMT_1, 0x00);
  writeMPU6050(GYRO_CONFIG, 0x00);
}

// Calibrate gyro (Z axis)
void calibrateMPU6050() {
  int numSamples = 1000;
  float sumZ = 0;
  for (int i = 0; i < numSamples; i++) {
    sumZ += readGyroZ();
    delay(3);
  }
  gyro_z_offset = sumZ / numSamples;
}

// Write value to MPU6050 register
void writeMPU6050(byte reg, byte value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read raw gyro Z-axis data and subtract offset
float readGyroZ() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDR, 2);
  int16_t rawZ = Wire.read() << 8 | Wire.read();
  return (rawZ / GYRO_SCALE) - gyro_z_offset;
}

// Calculate yaw error with angle wrapping
float calculateYawError(float targetYaw, float currentYaw) {
  float error = targetYaw - currentYaw;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  return error;
}



//////////////////////////////////////////////////////////
//          FreeRTOS Task for High-Frequency Gyro       //
//////////////////////////////////////////////////////////

// This task reads the gyro at ~200 Hz and updates the shared variable
void gyroTask(void * parameter) {
  while (true) {
    gyroZValue = readGyroZ();
    vTaskDelay(5 / portTICK_PERIOD_MS); // Delay ~5ms (~200 Hz sampling rate)
  }
}



//////////////////////////////////////////////////////////
//                Motor Functions                       //
//////////////////////////////////////////////////////////

// Setup motor driver pins and PWM
void motor_driver_setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL);
}

// Set motor speed and direction
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

// Stop the motor
void stop_motor() {
  move(-10);
  custom_delay(20);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CHANNEL, 0);
}

// Turn the robot by a specified degrees ('L' for left, 'R' for right)
void turn(char direction, int degrees) {
  if (direction == 'L' || direction == 'l') {
    targetYaw = yaw - degrees;
  } else if (direction == 'R' || direction == 'r') {
    targetYaw = yaw + degrees;
  } else {
    Serial.println("Invalid direction. Use 'L' or 'R'.");
    return;
  }
  if (targetYaw >= 360) targetYaw -= 360;
  if (targetYaw < 0) targetYaw += 360;

  if (debug) {
    Serial.print("Turning ");
    Serial.print((direction == 'L' || direction == 'l') ? "LEFT" : "RIGHT");
    Serial.print(" ");
    Serial.print(degrees);
    Serial.println(" degrees...");
  }
  
  // Set initial steering for turn
  if (direction == 'L' || direction == 'l') {
    steeringServo.write(STEERING_LEFT);
  } else {
    steeringServo.write(STEERING_RIGHT);
  }
  
  move(turn_speed);
  
  // Continue turning until within 10° of target yaw
  while (abs(targetYaw - yaw) > 10) {
    unsigned long current_time = millis();
    float dt = (current_time - prev_time) / 1000.0;
    prev_time = current_time;
    if (dt > 0.1) dt = 0.1;
    yaw -= gyroZValue * dt;
    if (yaw >= 360) yaw -= 360;
    if (yaw < 0) yaw += 360;
    move(turn_speed);
  }
  
  // Center steering after turn
  steeringServo.write(STEERING_CENTER);
  if (debug) {
    Serial.println("Turn complete. Centering steering.");
  }
}



//////////////////////////////////////////////////////////
//              Steering Functions                      //
//////////////////////////////////////////////////////////

// Initialize and test the steering servo
void steering_servo_setup() {
  steeringServo.attach(SteeringServoPin, 1000, 2000);
  steeringServo.write(STEERING_LEFT);
  custom_delay(500);
  steeringServo.write(STEERING_CENTER);
  custom_delay(500);
  steeringServo.write(STEERING_RIGHT);
  custom_delay(500);
  steeringServo.write(STEERING_CENTER);
  custom_delay(500);
}

// Set servo to a given angle
void steer(int steering_angle) {
  steeringServo.write(steering_angle);
}

// Update steering using PID based on target yaw
void update_steering_move(float targetYaw) {
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0;
  prev_time = current_time;
  if (dt > 0.1) dt = 0.1;

  // Use cached gyro value from the task
  float currentGyro = gyroZValue;
  yaw -= currentGyro * dt;
  if (yaw >= 360) yaw -= 360;
  if (yaw < 0) yaw += 360;

  pid_error = calculateYawError(targetYaw, yaw);
  pid_integral += pid_error * dt;
  // Anti-windup: limit the integral term
  pid_integral = constrain(pid_integral, -50, 50);
  float pid_derivative = (pid_error - last_pid_error) / dt;
  last_pid_error = pid_error;
  
  float correction = (kp * pid_error) + (ki * pid_integral) + (kd * pid_derivative);
  int steering_angle = STEERING_CENTER - correction;
  steering_angle = constrain(steering_angle, STEERING_RIGHT, STEERING_LEFT);
  steeringServo.write(steering_angle);

  if (debug) {
    Serial.print("Gyro: ");
    Serial.print(currentGyro);
    Serial.print(" | Yaw: ");
    Serial.print(yaw);
    Serial.print(" | Target: ");
    Serial.print(targetYaw);
    Serial.print(" | Error: ");
    Serial.print(pid_error);
    Serial.print(" | Integral: ");
    Serial.print(pid_integral);
    Serial.print(" | Derivative: ");
    Serial.print(pid_derivative);
    Serial.print(" | Correction: ");
    Serial.print(correction);
    Serial.print(" | Steering: ");
    Serial.println(steering_angle);
  }
}



//////////////////////////////////////////////////////////
//                Custom Functions                      //
//////////////////////////////////////////////////////////

// A simple busy-wait delay function
void custom_delay(long long delay_time) {
  long long start_time = millis();
  while (millis() - start_time < delay_time) {
    // Busy wait
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

void setup() {
  Wire.begin();
  Serial.begin(115200);
  cameraSerial.begin(19200);
  
  delay(2000);
  steering_servo_setup();
  motor_driver_setup();
  setupMPU6050();
  calibrateMPU6050();

  // Initialize yaw and set target yaw to current heading
  yaw = 0.0;
  targetYaw = yaw;
  prev_time = millis();
  
  // Start the gyro task on core 1
  xTaskCreatePinnedToCore(
    gyroTask,      // Task function
    "Gyro Task",   // Task name
    2048,          // Stack size in words
    NULL,          // Parameter
    1,             // Priority
    NULL,          // Task handle
    1              // Run on core 1
  );
}

void loop() {
  /* 
  if (cameraSerial.available() > 0) {
    char receivedChar = cameraSerial.read();
    if (receivedChar == '\n') {
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
      receivedMessage = "";
    } else {
      receivedMessage += receivedChar;
    }
  }
  */
  
  // Continuously update steering and drive the robot
  update_steering_move(targetYaw);
  move(robot_speed);
}
