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
  move(-200);
  delay(10);
  move(0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CHANNEL, 0);
}

// -----------------------------------------------------------------
// Helper: turn in place until you hit exactly target_angle
// -----------------------------------------------------------------
void move_until_angle(int speed, double target_angle) {
  // Reuse your steer_to_angle() logic
  steer_to_angle(target_angle, speed);
}

// -----------------------------------------------------------------
// Helper: drive forward target_cm while holding a given gyro angle
// -----------------------------------------------------------------
void move_cm_gyro(int target_cm, int speed, double hold_angle) {
  encoder_ticks = 0;
  move(speed);
  while (read_cm() < target_cm) {
    read_gyro_data();
    // PID to hold hold_angle
    double err = hold_angle - gz;
    pid_error = err * kp + (pid_error - pid_last_error) * kd;
    pid_last_error = err;
    steer(pid_error);
  }
  move(0);
}

void steer_to_angle(double target_angle, int speed) {
  read_gyro_data();
  double current_angle = gz;
  double error = target_angle - current_angle;
  int direction = (error >= 0) ? 1 : -1;

  pid_last_error = 0;
  move(speed);

  unsigned long start_time = millis();  // timeout start

  while (abs(error) >= 10.0) {  // max 1.5 sec turn
    read_gyro_data();
    error = target_angle - gz;

    pid_error = error * kp + (error - pid_last_error) * kd;
    pid_last_error = error;

    steer(pid_error * direction);
  }

  move(0);  // stop
}

// -----------------------------------------------------------
//           ISR & setup for single-edge counting
// -----------------------------------------------------------
void IRAM_ATTR encoderISR() {
  // On A’s rising edge, read B. If B==HIGH we’re turning “forward”
  if (digitalRead(ENCODER_B)) {
    encoder_ticks++;
  } else {
    encoder_ticks--;
  }
}

void encoder_setup() {
  // Enable pull-ups so A/B never float
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  // Attach only A’s rising edge:
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);
  encoder_ticks = 0;
}

float read_cm() {
  noInterrupts();
  int32_t t = encoder_ticks;
  interrupts();

  float dist_mm = t * MM_PER_TICK;
  float dist_cm = dist_mm / 10.0f;
  //Serial.printf("ticks=%ld  dist_cm=%.2f\n", t, dist_cm);
  return dist_cm;
}

void move_cm(int target_cm, int speed) {
  encoder_ticks = 0;
  move(speed);
  while (read_cm() < target_cm) {
    // your gyro‐PID steering here…
    read_gyro_data();
    double error = current_angle_gyro - gz;
    pid_error = error * kp + (pid_error - pid_last_error) * kd;
    pid_last_error = pid_error;
    steer(pid_error);
  }
}

// -----------------------------------------------------------
//                    Steering Functions
// -----------------------------------------------------------
void steering_servo_setup() {
  steeringServo.attach(STEERING_SERVO);
  steeringServo.write(STEERING_CENTER);
  delay(200);
}

void steer(double steering_angle) {
  steering_angle = map_double(steering_angle, -1, 1, STEERING_LEFT, STEERING_RIGHT);  // prevent over-rotation
  steeringServo.write(steering_angle);
}

void set_steer_angle(double steering_angle) {
  steeringServo.write(steering_angle);
}

void move_straight_on_gyro(double speed, long duration_ms) {
  int sign = speed >= 0 ? 1 : -1;

  unsigned long start_time = millis();
  while (millis() - start_time < duration_ms) {
    read_gyro_data();
    double err = current_angle_gyro - gz;  // try to hold a constant angle
    pid_error = (err)*kp + (pid_error - pid_last_error) * kd;
    pid_last_error = pid_error;

    steer(pid_error * sign);
    move(robot_speed);
    cameraSerial.flush();
  }
  move(0);  // stop
}