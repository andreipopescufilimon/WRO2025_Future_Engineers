#include "Wire.h"
#include "MPU6050.h"
#include "xmotionV3.h"

// Ultrasonic sensor pins
#define LeftTrigPin 2
#define LeftEchoPin 4
#define RightTrigPin 0
#define RightEchoPin 1

// Steering pins
#define SteeringServoPin 3 

// Control input pins
#define Start A0
#define DipSwitch1 5  
#define DipSwitch2 6  
#define DipSwitch3 7 

// PID Constants for Straight Movement
float KP = 0.7;
float KI = 0.001;
float KD = 1.5;
float integral = 0, previousError = 0;

// MPU6050 sensor
MPU6050 mpu;
float yawOffset = 0;  
float targetYaw = 0; 

// Lap and turn tracking
int turnCount = 0;
const int maxTurns = 12; 

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1)
      ;
  }

  // Get initial yaw offset
  yawOffset = getYaw();
  targetYaw = yawOffset; 

  // Set pin modes
  pinMode(LeftTrigPin, OUTPUT);
  pinMode(LeftEchoPin, INPUT);
  pinMode(RightTrigPin, OUTPUT);
  pinMode(RightEchoPin, INPUT);
  pinMode(Start, INPUT);
  pinMode(SteeringServoPin, OUTPUT); 

  for (int i = 0; i < 5; i++) {
      xmotion.ToggleLeds(200);
  }
   
  while (digitalRead(Start) == LOW) 
    ;  // Wait for Star Button to be pressed

  Serial.println("Start button pressed, starting 3 laps...");
  digitalWrite(UL1, HIGH);
  digitalWrite(UL2, HIGH);
}

void loop() {
  if (turnCount >= maxTurns) {
    Serial.println("Finished 3 laps!");
    xmotion.StopMotors(1000);
    
    while (1) {
      xmotion.ToggleLeds(100);  // Stop execution after completing 3 laps
    }
  }

  int leftDistance = measureDistance(LeftTrigPin, LeftEchoPin);
  int rightDistance = measureDistance(RightTrigPin, RightEchoPin);

  if (leftDistance > 50) {  // No left wall detected → Turn Left
    Serial.println("Turning Left 90°");
    turnRobot(90, 'L');
    turnCount++;
  } else if (rightDistance > 50) {  // No right wall detected → Turn Right
    Serial.println("Turning Right 90°");
    turnRobot(90, 'R');
    turnCount++;
  } else {
    moveStraightPID();
  }
}

// Function to move straight using PID control
void moveStraightPID() {
  float currentYaw = getYaw();
  float error = targetYaw - currentYaw;

  integral += error;
  float derivative = error - previousError;
  float correction = (KP * error) + (KI * integral) + (KD * derivative);
  previousError = error;

  // Adjust servo position based on correction (steering)
  int steeringAngle = constrain(90 + correction, 60, 120); 
  analogWrite(SteeringServoPin, steeringAngle);

  // Move forward at constant speed
  xmotion.Forward(80, 50);

  Serial.print("Yaw: ");
  Serial.print(currentYaw);
  Serial.print(" | Correction: ");
  Serial.println(correction);
}

// Function to make the robot turn 90° using the steering system
void turnRobot(int degrees, char direction) {
  float currentYaw = getYaw();
  float targetTurn = (direction == 'L') ? currentYaw - degrees : currentYaw + degrees;

  // Adjust steering servo for turn
  if (direction == 'L') {
    analogWrite(SteeringServoPin, 60);  // Turn left
  } else {
    analogWrite(SteeringServoPin, 120);  // Turn right
  }

  // Continue turning until desired yaw is reached
  while (abs(targetTurn - getYaw()) > 2) {
    xmotion.Forward(60, 50);
  }

  // Reset steering to go straight
  analogWrite(SteeringServoPin, 90);
  xmotion.StopMotors(200);

  targetYaw = getYaw();  // Update target yaw after turn
}

// Function to get yaw angle from MPU6050
float getYaw() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyro data to degrees per second
  float yawRate = gz / 131.0;
  float dt = 0.01;  // 10ms loop time
  return yawOffset + (yawRate * dt);
}

// Function to measure distance using an ultrasonic sensor
int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;  // Convert time to cm
  return distance;
}
