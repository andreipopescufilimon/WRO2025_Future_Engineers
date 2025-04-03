#include <Wire.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>

// ================ Pin Definitions ================
#define PWMA 11
#define STBY 10
#define AIN2 8
#define AIN1 7
#define START_BTN 4
#define STEERING_SERVO 2
#define P4 0
#define P5 1
#define BUILTIN_LED 13


// ================ Gyro Variables ================
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };

float YawAngle = 0;
float targetYaw = 1.0;  // 1 due to assimetry

// ================ Speed Control ================
int robot_speed = 100;  // 90
int turn_speed = 90;   // 80

// ================ Motors and PID ================
float kp = 0.32;  // 0.32
float ki = 0;
float kd = 8.5;  // 8.5
float pid_error = 0, last_pid_error = 0, pid_integral = 0;

#define PWM_CHANNEL 5
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

#define SteeringServoPin 2
Servo steeringServo;
#define STEERING_LEFT 115
#define STEERING_CENTER 79
#define STEERING_RIGHT 40

// ================ UART Communication ================
#define RX_PIN 0
#define TX_PIN 1
SoftwareSerial cameraSerial(RX_PIN, TX_PIN);
String receivedMessage = "";
char command = '0';

// ================ Turns count and direction ================
char turn_direction = '0';
int turn_count = 0;
const int max_turns = 12;
unsigned long lastTurnTime = 0;
const unsigned long turnCooldown = 2000;

// ================ Debug Mode ================
bool debug = false;
bool debuggyro = false;



// -----------------------------------------------------------
//                     Gyro Functions
// -----------------------------------------------------------
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

void calculate_gyro_data() {
  gyro_signals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  YawAngle += RateYaw * 0.004;

  if (YawAngle >= 360) YawAngle -= 360;
  else if (YawAngle < 0) YawAngle += 360;

  if (debuggyro) {
    Serial.print("Roll Angle [°] ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" Pitch Angle [°] ");
    Serial.print(KalmanAnglePitch);
    Serial.print(" Yaw Angle [°] ");
    Serial.println(YawAngle);
  }

  while (micros() - LoopTimer < 4000)
    ;
  LoopTimer = micros();
}

// -----------------------------------------------------------
//                     Motor Functions
// -----------------------------------------------------------
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
  ledcWrite(PWM_CHANNEL, 0);
}

// -----------------------------------------------------------
//                    Steering Functions
// -----------------------------------------------------------
void steering_servo_setup() {
  steeringServo.attach(SteeringServoPin);
  steeringServo.write(STEERING_CENTER);
  custom_delay(500);
}

void steer(int steering_angle) {
  steeringServo.write(steering_angle);
}

void turn(char direction, float degrees) {
  float initialYaw = YawAngle;
  float desiredYaw;

  if (initialYaw < 0) initialYaw += 360;

  if (direction == 'R') {
    steer(STEERING_RIGHT);
    desiredYaw = initialYaw - degrees;
    if (desiredYaw < 0) desiredYaw += 360;
  } else if (direction == 'L') {
    steer(STEERING_LEFT);
    desiredYaw = initialYaw + degrees;
    if (desiredYaw >= 360) desiredYaw -= 360;
  } else {
    stop_motor();
    steer(STEERING_CENTER);
    return;
  }

  move(turn_speed);
  while (true) {
    calculate_gyro_data();
    float currentYaw = YawAngle;
    if (currentYaw < 0) currentYaw += 360;

    float yawDiff = abs(currentYaw - desiredYaw);
    if (yawDiff > 180) yawDiff = 360 - yawDiff;
    if (yawDiff < 5.0) break;
  }

  targetYaw = desiredYaw;
  if (debug) {
    Serial.print("Completed turn. New target Yaw: ");
    Serial.println(targetYaw);
  }
}

void update_steering_move(float targetYaw) {
  float currentYaw = YawAngle;
  if (currentYaw < 0) currentYaw += 360;
  if (targetYaw < 0) targetYaw += 360;

  float yawError = targetYaw - currentYaw;
  if (yawError > 180) yawError -= 360;
  if (yawError < -180) yawError += 360;

  float yawDerivative = yawError - last_pid_error;

  pid_integral *= 0.95;
  pid_integral += yawError;

  float correction = kp * yawError + kd * yawDerivative + ki * pid_integral;
  correction *= 10;
  last_pid_error = yawError;

  int steeringAngle = STEERING_CENTER + correction;
  if (steeringAngle > STEERING_LEFT) steeringAngle = STEERING_LEFT;
  if (steeringAngle < STEERING_RIGHT) steeringAngle = STEERING_RIGHT;

  steer(steeringAngle);

  if (debug) {
    Serial.print("TargetYaw: ");
    Serial.print(targetYaw);
    Serial.print(" | CurrentYaw: ");
    Serial.print(currentYaw);
    Serial.print(" | Error: ");
    Serial.print(yawError);
    Serial.print(" | Steering Angle: ");
    Serial.println(steeringAngle);
  }
}

// -----------------------------------------------------------
//                           Customs
// -----------------------------------------------------------
void custom_delay(long long delay_time) {
  long long start_time = millis();
  while (millis() - start_time < delay_time) {}
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

// -----------------------------------------------------------
//                           Main
// -----------------------------------------------------------
void setup() {
  delay(1500);

  steering_servo_setup();
  motor_driver_setup();
  cameraSerial.begin(19200);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 4000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  RateCalibrationRoll /= 4000;
  RateCalibrationPitch /= 4000;
  RateCalibrationYaw /= 4000;

  LoopTimer = micros();

  if (debug) Serial.begin(115200);

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);

  pinMode(START_BTN, INPUT_PULLUP);
  while (digitalRead(START_BTN) == LOW)
    ;
  digitalWrite(BUILTIN_LED, LOW);
  custom_delay(500);
}

void loop() {
  if (turn_count >= max_turns) {
    unsigned long t = millis();
    while (millis() - t < 800) {
      calculate_gyro_data();
      update_steering_move(targetYaw);
      move(robot_speed);
    }
    stop_motor();
    if (debug) Serial.println("Maximum turns reached. Stopping...");
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

  calculate_gyro_data();
  update_steering_move(targetYaw);
  move(robot_speed);
}
