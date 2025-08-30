#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include "BMI088.h"

#define RUN_MODE 1  // 0 - Open Round, 1 - Cubes Round

// ================ Pin Definitions ================
#define PWMA D7
#define DIRA D8

#define PWM_IMPELLER D9

#define START_BTN D10

#define P4 D0
#define P5 D1

#define DEBUG_LED1 D6
#define DEBUG_LED2 D3

#define PWM_DIST_FRONT A0
#define PWM_DIST_BACK A2
#define PWM_DIST_LEFT A3
#define PWM_DIST_RIGHT A1

// ======= Distance sensor direction enum =======
enum DistanceDir : uint8_t { FRONT_DIR,
                             LEFT_DIR,
                             RIGHT_DIR,
                             BACK_DIR };


// ================ Speed Control ================
int robot_speed = 56;  // 57 - 59 // 100
int exit_speed = 56;
int park_speed = 58;
int impeller_speed = 0;
int cube_last = 1;

// === Encoder pins ===
#define ENCODER_A D5
#define ENCODER_B D4
volatile int32_t encoder_ticks = 0;

#define PPR 12.0f  // 12 pulses per rev, single-edge
#define QUAD_EDGES (PPR * 4.0f)
#define GEAR_RATIO 1.0f
#define WHEEL_DIAM 29.0f  // mm
#define MM_PER_TICK 1.18f

// ================ Motors and PID ================3
double current_angle_gyro = 0.0;  // steering asymmetry
double kp = 0.038;                // 0.044
double ki = 0;
double kd = 0.061;  // 0.082
double pid_error = 0, pid_last_error = 0;

// Integration & drift
double gyro_last_read_time = 0; 
double drifts_z = 0;  // drift in °/s
double gz = 0;        // integrated angle in degrees


#define PWM_MOTOR_CHANNEL 5
#define PWM_MOTOR_FREQ 1000
#define PWM_MOTOR_RESOLUTION 8

#define PWM_IMPELLER_CHANNEL 4
#define PWM_IMPELLER_FREQ 1000
#define PWM_IMPELLER_RESOLUTION 8


#define STEERING_SERVO D2
Servo steeringServo;
#define STEERING_LEFT 135
#define STEERING_CENTER 85
#define STEERING_RIGHT 35

#define CORRECTION_ANGLE 35

#define FOLLOW_CUBE_DEAD_TIME 250

// ================ UART Communication ================
#define RX_PIN D0
#define TX_PIN D1
SoftwareSerial cameraSerial(RX_PIN, TX_PIN);
String receivedMessage = "";
String command = "";
String current_color = "";

// ================ Turns count and direction ================
int turn_direction = 0;
int turn_count = 0;
const int max_turns = 12;
unsigned long lastTurnTime = 0, lastLineDetectedTime = 0;

long last_follow_cube = 0;


// ================ Avoid Cubes Variables ================
float follow_cube_angle = 0;  // PID steering correction for cube following.

unsigned long last_cube_time = 0;  // remember when we last avoided one


// ================ Robot States ================
enum RobotState {
  PID,
  FOLLOW_CUBE,
  AVOID_CUBE,
  AFTER_CUBE,
  PARK
};

RobotState currentState = PID;
char cube_avoid_direction = 'R';  // 'R' for red cube avoidance, 'L' for green cube avoidance
float desiredSteering = 0.0;
 
// ================ Debug Mode ================
bool debug = false;
bool debuggyro = false;
bool debugcam = false;
bool firstcube = false;


void setup() {

  //debug_distance_infinite(LEFT_DIR);

  steering_servo_setup();
  motor_driver_setup();
  impeller_setup();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DEBUG_LED1, OUTPUT);
  pinMode(DEBUG_LED2, OUTPUT);

  pinMode(PWM_DIST_LEFT, INPUT);
  pinMode(PWM_DIST_RIGHT, INPUT);
  pinMode(PWM_DIST_FRONT, INPUT);
  pinMode(PWM_DIST_BACK, INPUT);


  encoder_setup();

  Serial.begin(115200);

  cameraSerial.begin(19200);
  while (!cameraSerial)
    ;
  blink_led(DEBUG_LED1, 200);

  gyro_setup();
  blink_led(DEBUG_LED2, 200);

  digitalWrite(DEBUG_LED2, HIGH);
  pinMode(START_BTN, INPUT_PULLUP);

  while (digitalRead(START_BTN) == 1) {
    move(0);
    flush_messages();
  }

  //move_gyro_until_side_detect_cycle(LEFT_DIR, park_speed, current_angle_gyro);

  setImpeller(impeller_speed);
  delay(500);

  digitalWrite(DEBUG_LED1, HIGH);
  custom_delay(500);

  // Start from parking
  if (RUN_MODE == 1) {
    if (readDistanceMM(LEFT_DIR) > readDistanceMM(RIGHT_DIR)) {  // exit right/.
      move_until_angle_max(exit_speed, 75);

      move(65);
      delay(100);

      turn_direction = 1;
      move_until_angle_max(exit_speed, 0);

      move_straight_on_gyro(-robot_speed, 1400);
    } else {  // exit left
      move_until_angle_max(exit_speed, -70);

      move(65);
      delay(100);

      turn_direction = -1;

      move_until_angle_max(exit_speed, 0);

      move_straight_on_gyro(-robot_speed, 1800);
    }
  }
}

void loop() {

  // 1) Max‐turns timeout
  if (turn_count >= max_turns || current_angle_gyro == 1080) {
    if (RUN_MODE == 1) {
      currentState = PARK;
    } else {
      unsigned long t0 = millis();
      while (millis() - t0 < 1400) {
        read_gyro_data();
        double error = current_angle_gyro - gz;
        pid_error = error * kp + (pid_error - pid_last_error) * kd;
        pid_last_error = pid_error;
        steer(pid_error);
        move(robot_speed);
      }
      stop_motor();
      if (debug) Serial.println("Maximum turns reached. Stopping...");
      while (true)
        ;
    }
  }

  // 2) Fresh gyro read and flush camera
  read_gyro_data();

  // 3) State‐machine drive logic
  switch (currentState) {

    case PID:
      {
        // PID straight‐drive
        double err = current_angle_gyro - gz;
        pid_error = (err)*kp + (pid_error - pid_last_error) * kd;
        pid_last_error = pid_error;
        steer(pid_error);
        move(robot_speed);
        break;
      }

    case FOLLOW_CUBE:
      {
        if (millis() - last_follow_cube > FOLLOW_CUBE_DEAD_TIME) {
          currentState = PID;
        }
        // steer toward cube
        steer(follow_cube_angle);
        move(robot_speed);
        break;
      }

    case AVOID_CUBE:
      {
        // avoidance maneuver
        pass_cube(cube_avoid_direction);
        break;
      }

    case AFTER_CUBE:
      {
        int angle_addition = (cube_last == 1) ? 0 : 0;
        double err = current_angle_gyro - gz + cube_last * (CORRECTION_ANGLE + angle_addition);
        if (abs(err) < 5) {
          if (-cube_last == turn_direction) {
            move_cm(5, robot_speed, current_angle_gyro + -1 * cube_last * (CORRECTION_ANGLE + angle_addition));
          } else {
            move_cm(5, robot_speed, current_angle_gyro + -1 * cube_last * (CORRECTION_ANGLE + angle_addition));
          }
          currentState = PID;
        } else {
          pid_error = (err)*kp + (pid_error - pid_last_error) * kd;
          pid_last_error = pid_error;
          steer(pid_error);
        }
        move(robot_speed);
        //flush_messages();
        break;
      }

    case PARK:
      {
        // in functions.ino
        break;
      }
  }

  // 4) Read & dispatch any new OpenMV commands
  while (cameraSerial.available() > 0) {
    char c = cameraSerial.read();
    if (c == '\n') {
      // Normalize & execute
      receivedMessage.trim();
      receivedMessage.toUpperCase();
      execute_command(receivedMessage);
      receivedMessage = "";
      flush_messages();
    } else {
      receivedMessage += c;
    }
  }

  //Serial.println(currentState);
}