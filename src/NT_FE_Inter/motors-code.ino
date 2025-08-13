// -----------------------------------------------------------
//                     Motor Functions
// -----------------------------------------------------------
void motor_driver_setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA, OUTPUT);
  ledcSetup(PWM_MOTOR_CHANNEL, PWM_MOTOR_FREQ, PWM_MOTOR_RESOLUTION);
  ledcAttachPin(PWMA, PWM_MOTOR_CHANNEL);
  move(0);
}

void move(int speed) {
  ledcWrite(PWM_MOTOR_CHANNEL, abs(speed));
  digitalWrite(DIRA, speed > 0 ? LOW : HIGH);
}

void impeller_setup() {
  pinMode(PWM_IMPELLER, OUTPUT);
  ledcSetup(PWM_IMPELLER_CHANNEL, PWM_IMPELLER_FREQ, PWM_IMPELLER_RESOLUTION);
  ledcAttachPin(PWM_IMPELLER, PWM_IMPELLER_CHANNEL);
}

void setImpeller(int _pwm) {
  ledcWrite(PWM_IMPELLER_CHANNEL, constrain(_pwm, 0, 255));
}

void stop_motor() {
  move(-3);
  delay(100);
  move(0);
}

// -----------------------------------------------------------------
// Helper: turn in place until you hit exactly target_angle
// -----------------------------------------------------------------
void move_until_angle(int speed, double target_angle) {
  int dir = (speed < 0) ? -1 : 1;

  read_gyro_data();
  double error = target_angle - gz;
  while (abs(error) >= 10) {
    read_gyro_data();
    error = target_angle - gz;
    pid_error = (error)*kp + (pid_error - pid_last_error) * kd;
    pid_last_error = pid_error;
    steer(pid_error * dir);
    move(robot_speed);
    flush_messages();
  }
}

void move_until_angle_max(int speed, double target_angle) {
    int dir = (speed < 0) ? -1 : 1;

    read_gyro_data();
    double error = target_angle - gz;
    move(speed);
    while (fabs(error) >= 10.0) {
        read_gyro_data();
        error = target_angle - gz;
        int steer_cmd;
        if (error * dir > 0) {
            steer_cmd =  1;
        } else {
            steer_cmd = -1;
        }
        steer(steer_cmd);
        move(speed);
        flush_messages();
    }
}


// -----------------------------------------------------------------
// Helper: drive forward target_cm while holding a given gyro angle
// -----------------------------------------------------------------
void move_cm_gyro(int target_cm, int speed, double hold_angle) {
  int dir = (speed < 0) ? -1 : 1;

  encoder_ticks = 0;
  move(speed);
  while (read_cm() < target_cm) {
    read_gyro_data();
    // PID to hold hold_angle
    double err = hold_angle - gz;
    pid_error = err * kp + (pid_error - pid_last_error) * kd;
    pid_last_error = err;
    steer(pid_error * dir);
    flush_messages();
  }
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

void move_cm(int target_cm, int speed, double gyro_offset) {
  noInterrupts();
  encoder_ticks = 0;
  interrupts();

  pid_last_error = 0;
  pid_error = 0;

  move(speed);
  while (read_cm() < target_cm) {
    read_gyro_data();
    double error = gyro_offset - gz;
    pid_error = error * kp + ((error - pid_last_error) * kd);
    pid_last_error = error;

    double sign = (speed < 0) ? -1.0 : 1.0;
    steer(pid_error * sign);

    move(speed);
    flush_messages();
  }
}


// -----------------------------------------------------------
//                    Steering Functions
// -----------------------------------------------------------
void steering_servo_setup() {
  steeringServo.attach(STEERING_SERVO);
  delay(100);

  steeringServo.write(STEERING_LEFT);
  delay(300);
  steeringServo.write(STEERING_RIGHT);
  delay(300);
  steeringServo.write(STEERING_CENTER);
  delay(300);
}

void steer(double steering_angle) {
  // Clamp input between -1 and 1
  if (steering_angle > 1)  steering_angle = 1;
  if (steering_angle < -1) steering_angle = -1;

  // Map to servo range
  steering_angle = map_double(steering_angle, -1, 1, STEERING_LEFT, STEERING_RIGHT);
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
    move(speed);
    cameraSerial.flush();
    flush_messages();
  }
}