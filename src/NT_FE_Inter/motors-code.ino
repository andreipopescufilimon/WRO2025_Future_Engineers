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

// ===========================================================
// Move until distance sensor reads target distance (front/back)
// Maintains a target heading using the gyro.
// Keeps running until the distance is within tolerance band.
// ===========================================================
void move_to_distance(DistanceDir dir, float target_mm, float tol_mm, int base_speed, double hold_angle)
{
  // speed plan
  const int MIN_SPEED   = 50;                                // must be enough to move
  const int MAX_SPEED   = 60;     // cap by caller
  const int SEARCH_SPEED = MIN_SPEED;                         // speed while sensor = -1

  // FRONT sensor "forward" is + ; BACK sensor "forward" is -
  const int forward_sign = (dir == FRONT_DIR) ? +1 : -1;

  pid_last_error = 0;
  pid_error = 0;

  while (true) {
    // --- Read sensor ---
    float d_mm = readDistanceMM(dir);

    if (d_mm < 0) {
      // No measurement: move in the forward direction for this sensor
      read_gyro_data();
      double err_ang = hold_angle - gz;
      pid_error = (err_ang) * kp + ((err_ang - pid_last_error) * kd);
      pid_last_error = err_ang;

      // keep steering consistent when moving backward
      double steer_cmd = pid_error * (forward_sign >= 0 ? 1.0 : -1.0);
      steer(steer_cmd);

      move(forward_sign * SEARCH_SPEED);
      flush_messages();
      continue; // try reading again
    }

    // --- We have a valid distance: control to target ---
    float delta = d_mm - target_mm;   // >0 too far, <0 too close
    if (fabs(delta) <= tol_mm) {
      move(0);
      break;                          // done
    }

    // Decide direction to reduce error:
    // FRONT: too far → forward (+) ; BACK: too far → backward (–)
    int error_sign     = (delta > 0) ? +1 : -1;
    int commanded_sign = forward_sign * error_sign;

    // Proportional speed magnitude (clamped)
    float mag = map_double(fminf(fabs(delta), 300.0f), 0, 300.0, MIN_SPEED, MAX_SPEED);
    int speed_cmd = commanded_sign * int(mag);

    // --- Gyro hold ---
    read_gyro_data();
    double err_ang = hold_angle - gz;
    pid_error = (err_ang) * kp + ((err_ang - pid_last_error) * kd);
    pid_last_error = err_ang;

    // Flip steering when going backward so correction stays intuitive
    double steer_cmd = pid_error * (speed_cmd >= 0 ? 1.0 : -1.0);
    steer(steer_cmd);

    move(speed_cmd);
    flush_messages();
  }
}


// --- Common detect rule: wall is seen only if 1 < d < 300 (mm)
static inline bool is_wall_seen(float d_mm) {
  return (d_mm > 5.0f && d_mm < 200.0f);
}


// Move straight (gyro hold) until side sensor goes:
// DETECT  ->  NO DETECT  ->  DETECT (single edge transitions, no multi-sample debounce)
void move_gyro_until_side_detect_cycle(DistanceDir side, int speed, double hold_angle)
{
  digitalWrite(DEBUG_LED1, LOW);
  digitalWrite(DEBUG_LED2, LOW);
  if (!(side == LEFT_DIR || side == RIGHT_DIR)) return;

  pid_last_error = 0;
  pid_error = 0;

  uint8_t phase = 0;        // 0: wait DETECT, 1: wait NO DETECT, 2: wait DETECT -> stop
  bool last_detected = false;
  bool first_read    = true;

  move(speed);

  while (true) {
    // --- Gyro hold ---
    read_gyro_data();
    double err = hold_angle - gz;
    pid_error = err * kp + (err - pid_last_error) * kd;
    pid_last_error = err;

    int dir = (speed < 0) ? -1 : 1;
    steer(pid_error * dir);
    move(speed);

    // --- Single read & classify using the rule 1 < d < 300 ---
    float d = readDistanceMM(side);
    bool detected = is_wall_seen(d);

    if (first_read) { last_detected = detected; first_read = false; }

    // --- Phase machine on single edges ---
    if (phase == 0) {
      if (!last_detected && detected) {           // rising edge
        phase = 1;
      }
    } else if (phase == 1) {
      digitalWrite(DEBUG_LED1, HIGH);
      if (last_detected && !detected) {           // falling edge
        phase = 2;
      }
    } else { // phase == 2
      digitalWrite(DEBUG_LED2, HIGH);
      if (!last_detected && detected) {           // rising edge -> done
        break;
      }
    }

    last_detected = detected;
    // optional: small yield if needed
    // delay(1);
  }

  move(0);
  steer(0);
}