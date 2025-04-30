#include <Wire.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include "BMI088.h"  // New: BMI088 library header

#define RUN_MODE 0  // 0 - Open Round, 1 - Cubes Round

// ================ Pin Definitions ================
#define PWMA D11
#define STBY D10
#define AIN2 D8
#define AIN1 D7
#define START_BTN D4
#define STEERING_SERVO D2
#define P4 D0
#define P5 D1
#define DEBUG_LED D12

// ================ Speed Control ================
int robot_speed = 90;  // 90
int turn_speed = 80;   // 80
int follow_speed = 90;

// ================ Motors and PID ================
double current_angle_gyro = 1.25;  // steering asymmetry
double kp = 0.025;
double ki = 0;
double kd = 0.042;
double pid_error = 0, pid_last_error = 0;

// Integration & drift
double gyro_last_read_time = 0;
double drifts_x = 0;  // drift in °/s
double gx = 0;        // integrated angle in degrees


#define PWM_CHANNEL 5
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

#define STEERING_SERVO 2
Servo steeringServo;
#define STEERING_LEFT 115
#define STEERING_CENTER 79
#define STEERING_RIGHT 40

#define STEERING_AVOID_LEFT 120  // Max left for avoidance maneuvers
#define STEERING_AVOID_RIGHT 30  // Max right for avoidance maneuvers

#define STEERING_LEFT_PARK 130
#define STEERING_CENTER_PARK 79
#define STEERING_RIGHT_PARK 25

// ================ UART Communication ================
#define RX_PIN 0
#define TX_PIN 1
SoftwareSerial cameraSerial(RX_PIN, TX_PIN);
String receivedMessage = "";
String command = "";
String current_color = "";

// ================ Turns count and direction ================
char turn_direction = 0;
int turn_count = 0;
const int max_turns = 12;
unsigned long lastTurnTime = 0, lastLineDetectedTime = 0;
const unsigned long turnCooldown = 2000;

// ================ Avoid Cubes Variables ================
#define AVOIDANCE_ANGLE 35       // Base avoidance angle in degrees.
#define AVOIDANCE_DRIVE_TIME 15  // Time (ms) to drive forward during avoidance.
float follow_cube_angle = 0;     // PID steering correction for cube following.

#define EXIT_MOVE_TIME 425    // Duration (ms) for the aggressive exit move
#define RETURN_MOVE_TIME 530  // Duration (ms) for the aggressive return move

// ================ Robot States ================
enum RobotState {
  DEFAULT_CASE,
  FOLLOW_CUBE,
  AVOID_CUBE,
  AFTER_CUBE
};

RobotState currentState = DEFAULT_CASE;
char cube_avoid_direction = 'R';  // 'R' for red cube avoidance, 'L' for green cube avoidance
int desiredSteering;

// ================ Debug Mode ================
bool debug = false;
bool debuggyro = true;
bool debugcam = false;
bool firstcube = false;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);

  steering_servo_setup();
  motor_driver_setup();

  cameraSerial.begin(19200);
  while (!cameraSerial)
    ;
  blink_led(LED_BUILTIN, 500);

  gyro_setup();

  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(START_BTN, INPUT_PULLUP);
  while (digitalRead(START_BTN) == LOW)
    ;
  digitalWrite(LED_BUILTIN, LOW);
  custom_delay(500);

  if (RUN_MODE == 1) {
  }
}

void loop() {
  if (turn_count >= max_turns) {
    unsigned long current_time = millis();
    while (millis() - current_time < 800) {
      read_gyro_data();
      double error = current_angle_gyro - gx;
      pid_error = (error)*kp + (pid_error - pid_last_error) * kd;
      pid_last_error = pid_error;
      steer(pid_error);
      move(robot_speed);
    }
    stop_motor();
    if (debug) Serial.println("Maximum turns reached. Stopping...");
    while (true)
      ;
  }

  command = "";
  current_color = "";
  while (cameraSerial.available() > 0) {
    char receivedChar = cameraSerial.read();
    if (receivedChar == '\n') {
      if (receivedMessage.startsWith("S")) {
        command = receivedMessage;
        if (debugcam) {
          Serial.print("Received from OpenMV: ");
          Serial.println(command);
        }
      } else {
        if (debugcam) {
          Serial.print("Received from OpenMV: ");
          Serial.println(receivedMessage);
        }
        receivedMessage.toUpperCase();
        if (receivedMessage.indexOf("BLACK") != -1) {
          command = "B";
        } else if (receivedMessage.indexOf("BLUE") != -1) {
          command = "L";
          if (turn_direction == 0) turn_direction = -1;
        } else if (receivedMessage.indexOf("ORANGE") != -1) {
          command = "O";
          if (turn_direction == 0) turn_direction = 1;
        } else if (receivedMessage.indexOf("RED") != -1) {
          command = "R";
        } else if (receivedMessage.indexOf("GREEN") != -1) {
          command = "G";
        } else if (receivedMessage.indexOf("PINK") != -1) {
          command = "P";
        } else if (receivedMessage.indexOf("FR") != -1) {
          command = "F";
          current_color = "R";
        } else if (receivedMessage.indexOf("FG") != -1) {
          command = "F";
          current_color = "G";
        } else {
          command = receivedMessage;
        }
      }
      if (debugcam) {
        Serial.print("Command received from OpenMV: ");
        Serial.println(command);
      }
      execute_command(command);
      receivedMessage = "";
      cameraSerial.flush();
    } else {
      receivedMessage += receivedChar;
    }
  }

  if (currentState == DEFAULT_CASE) {
    read_gyro_data();
    double error = current_angle_gyro - gx;
    pid_error = (error)*kp + (pid_error - pid_last_error) * kd;
    pid_last_error = pid_error;
    steer(pid_error);
    move(robot_speed);
  }
}