#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include "BMI088.h"  // New: BMI088 library header

#define RUN_MODE 1  // 0 - Open Round, 1 - Cubes Round

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
#define DIST_SENSOR D3

// ================ Speed Control ================
int robot_speed = 110;  // 200 - 240 in qualy, max 255 but with high drift on turns | stable 100
int turn_speed = 110;
int follow_speed = 90;  // not used

// === Encoder pins ===
#define ENCODER_A D5
#define ENCODER_B D6
volatile int32_t encoder_ticks = 0;

#define PPR 12.0f  // 12 pulses per rev, single-edge
#define QUAD_EDGES (PPR * 4.0f)
#define GEAR_RATIO 1.0f
#define WHEEL_DIAM 56.0f  // mm
#define MM_PER_TICK 1.2979f

// ================ Motors and PID ================
double current_angle_gyro = 0.0;  // steering asymmetry
double kp = 0.048;
double ki = 0;
double kd = 0.072;
double pid_error = 0, pid_last_error = 0;

// Integration & drift
double gyro_last_read_time = 0;
double drifts_z = 0;  // drift in °/s
double gz = 0;        // integrated angle in degrees


#define PWM_CHANNEL 5
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8


#define STEERING_SERVO 2
Servo steeringServo;
#define STEERING_LEFT 120
#define STEERING_CENTER 79
#define STEERING_RIGHT 30

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
int turn_direction = 0;
int turn_count = 0;
const int max_turns = 12;
unsigned long lastTurnTime = 0, lastLineDetectedTime = 0;
const unsigned long turnCooldown = 4000;

// ================ Avoid Cubes Variables ================
#define AVOIDANCE_ANGLE 35       // Base avoidance angle in degrees.
#define AVOIDANCE_DRIVE_TIME 15  // Time (ms) to drive forward during avoidance.
float follow_cube_angle = 0;     // PID steering correction for cube following.

#define EXIT_MOVE_TIME 350    // Duration (ms) for the aggressive exit move
#define RETURN_MOVE_TIME 300  // Duration (ms) for the aggressive return move

// ================ Robot States ================
enum RobotState {
  DEFAULT_CASE,
  FOLLOW_CUBE,
  AVOID_CUBE,
  AFTER_CUBE
};

RobotState currentState = DEFAULT_CASE;
char cube_avoid_direction = 'R';  // 'R' for red cube avoidance, 'L' for green cube avoidance
float desiredSteering = 0.0;

// ================ Debug Mode ================
bool debug = false;
bool debuggyro = false;
bool debugcam = false;
bool firstcube = false;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);

  steering_servo_setup();
  motor_driver_setup();
  encoder_setup();

  Serial.begin(115200);

  cameraSerial.begin(19200);
  while (!cameraSerial)
    ;
  blink_led(LED_BUILTIN, 200);

  gyro_setup();
  blink_led(LED_BUILTIN, 200);

  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(START_BTN, INPUT_PULLUP);
  while (digitalRead(START_BTN) == LOW)
    ;
  digitalWrite(LED_BUILTIN, LOW);
  custom_delay(500);

  // Start from parking
  /*
  if (RUN_MODE == 1) {
    // --- Parameters to tune ---
    float ENTRY_ROTATION_DEG = 75.0f;  // initial turn to exit the parking slot
    const int FORWARD_EXIT_CM = 2;          // how far to drive forward after exit
    const int RED_PARK_FORWARD_CM = 2;      // small forward bump on red detection
    const int GREEN_APPROACH_CM = 5;         // initial approach on green detection
    const int GREEN_TURN_CM = 15;            // lateral shift distance in green maneuver
    const int GREEN_STRAIGHT_CM = 25;        // straight-through distance in green maneuver
    const int GREEN_BACK_OUT_CM = 15;        // reverse out distance in green maneuver
    const int EXIT_SPEED = 80;             // PWM speed for all moves

    // read which side we're approaching from
    int way = digitalRead(DIST_SENSOR);

    // 1) Initial steer into the slot, then drive in by a fixed distance
    if (way == 1) {
      ENTRY_ROTATION_DEG = 75.0;
    } else {
      ENTRY_ROTATION_DEG = -75.0;
    }
    steer_to_angle(ENTRY_ROTATION_DEG, EXIT_SPEED);  // rotate to entry angle
    move_cm(FORWARD_EXIT_CM, EXIT_SPEED);          // drive in a bit

    // 2) Wait for the camera to send a line of text
    command = "";
    while (cameraSerial.available() > 0) {
      char c = cameraSerial.read();
      if (c == '\n') {
        receivedMessage.trim();
        receivedMessage.toUpperCase();

        if (receivedMessage.startsWith("S")) {
          // Special “S” command from OpenMV
          command = receivedMessage;
          if (debugcam) {
            Serial.print("CMD from OpenMV: ");
            Serial.println(command);
          }

        } else {
          // Color-based parking maneuver
          if (debugcam) {
            Serial.print("Color from OpenMV: ");
            Serial.println(receivedMessage);
          }

          if (receivedMessage.indexOf("RED") != -1) {
            // RED: just a small forward bump, then stop
            if (way == 0) {
              set_steer_angle(STEERING_RIGHT_PARK);
            } else {
              set_steer_angle(STEERING_LEFT_PARK);
            }
            move_cm(RED_PARK_FORWARD_CM, EXIT_SPEED);

          } else if (receivedMessage.indexOf("GREEN") != -1) {
            // GREEN: a little approach, then an S-shaped maneuver
            move_cm(GREEN_APPROACH_CM, EXIT_SPEED);

            if (way == 0) {
              // first curve to the left
              set_steer_angle(STEERING_LEFT_PARK);
              move_cm(GREEN_TURN_CM, EXIT_SPEED);

              // then curve back to the right and go through
              set_steer_angle(STEERING_RIGHT_PARK);
              move_cm(GREEN_STRAIGHT_CM, EXIT_SPEED);

            } else {
              // mirror for the opposite side
              set_steer_angle(STEERING_RIGHT_PARK);
              move_cm(GREEN_TURN_CM, EXIT_SPEED);

              set_steer_angle(STEERING_LEFT_PARK);
              move_cm(GREEN_STRAIGHT_CM, EXIT_SPEED);
            }

            // finally back out a bit
            move_cm(GREEN_BACK_OUT_CM, -EXIT_SPEED);
          }
        }

        // clear for next message
        receivedMessage = "";
        cameraSerial.flush();
      } else {
        receivedMessage += c;
      }
    }
  }
  */
}

void loop() {
  if (turn_count >= max_turns) {
    unsigned long current_time = millis();
    while (millis() - current_time < 700) {
      read_gyro_data();
      double error = current_angle_gyro - gz;
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

  read_gyro_data();
  double error = current_angle_gyro - gz;
  pid_error = (error)*kp + (pid_error - pid_last_error) * kd;
  pid_last_error = pid_error;
  if (debug == true) {
    Serial.print(current_angle_gyro);
    Serial.print("     |      ");
    Serial.print(turn_direction);
    Serial.print("     |      ");
    Serial.print(gz);
    Serial.print("     |      ");
    Serial.println(robot_speed);
  }
  steer(pid_error);
  move(robot_speed);
}