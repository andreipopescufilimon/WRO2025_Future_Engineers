#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include "BMI088.h" 

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
#define FRONT_ULTRASONIC_PIN A3
#define BACK_ULTRASONIC_PIN A0

// ================ Speed Control ================
int robot_speed = 85;  // 200 - 240 in qualy, max 255 but with high drift on turns | stable 100
int cube_last = 1;

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
double current_angle_gyro = -1.0;  // steering asymmetry
double kp = 0.032;                 // 0.044
double ki = 0;
double kd = 0.051;  // 0.082
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
#define STEERING_LEFT 130
#define STEERING_CENTER 82
#define STEERING_RIGHT 30

#define CORRECTION_ANGLE 50

#define FOLLOW_CUBE_DEAD_TIME 250

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
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);

  setupUltrasonic(FRONT_ULTRASONIC_PIN);
  setupUltrasonic(BACK_ULTRASONIC_PIN);

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

  while (digitalRead(START_BTN) == 1) {
    move(0);
    flush_messages();
  }

  digitalWrite(LED_BUILTIN, LOW);
  custom_delay(500);

  pinMode(DIST_SENSOR, INPUT);
  // Start from parking
  if (RUN_MODE == 1) {
    if (digitalRead(DIST_SENSOR) == 1) { // exit right
      move_until_angle_max(75, 80);
      turn_direction = 1;
      move_until_angle_max(75, 0);
    } else { // exit left
      move_until_angle_max(75, -80);
      turn_direction = -1;
      move_until_angle_max(75, 0);
    }
  }
}

void loop() {
  // 1) Max‐turns timeout
  if (turn_count >= max_turns) {
    if (RUN_MODE == 1) {
      currentState = PARK;
    } else {
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
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

  //if (abs(gz - current_angle_gyro) >= 89) current_angle_gyro += 90;

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
        int angle_addition = (cube_last == 1) ? -9 : 0;
        double err = current_angle_gyro - gz + cube_last * (CORRECTION_ANGLE + angle_addition);
        if (abs(err) < 10) {
          if (-cube_last == turn_direction) {
            move_cm(5, robot_speed, current_angle_gyro + cube_last * (CORRECTION_ANGLE + angle_addition));
          } else {
            move_cm(10, robot_speed, current_angle_gyro + cube_last * (CORRECTION_ANGLE + angle_addition));
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

    case PARK: {
      double err = current_angle_gyro - gz - turn_direction * 90;
      pid_error = (err) * kp + (pid_error - pid_last_error) * kd;
      pid_last_error = pid_error;
      steer(pid_error);
      move(60);
      delay(500);
      move(0);
      delay(20000);
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
