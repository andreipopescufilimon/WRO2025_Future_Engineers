#define AVOIDANCE_ANGLE 48  // how far to pivot around the cube
int delayTurns = 500;

void flush_messages() {
  while (cameraSerial.available() > 0) {
    char receivedChar = cameraSerial.read();
    if (receivedChar == '\n') {
      receivedMessage = "";
    } else {
      receivedMessage += receivedChar;
    }
  }
}


// -----------------------------------------------------------
//                Pololu Distance Sensor
// -----------------------------------------------------------

// ======= Pololu PWM Distance Sensor Settings =======
static const float PW_OFFSET_US   = 1000.0f; // microseconds offset at ~0 mm
static const float PW_US_PER_MM   = 1.0f;    // microseconds per millimeter
static const unsigned long PULSE_TIMEOUT_US = 30000UL; // 30 ms timeout

// ======= Map direction to the correct pin =======
static inline int distanceSensorPin(DistanceDir d) {
  switch (d) {
    case FRONT_DIR: return PWM_DIST_FRONT; // A0
    case LEFT_DIR:  return PWM_DIST_LEFT;  // A3
    case RIGHT_DIR: return PWM_DIST_RIGHT; // A1
    case BACK_DIR:  return PWM_DIST_BACK;  // A2
  }
  return PWM_DIST_FRONT;
}

// ======= Read pulse in microseconds =======
static inline unsigned long readPulseUS(int pin) {
  pinMode(pin, INPUT);
  return pulseIn(pin, HIGH, PULSE_TIMEOUT_US);
}

// ======= Convert pulse width to millimeters =======
static inline float pulseToMM(unsigned long pw_us) {
  if (pw_us == 0) return -1.0f; // timeout
  float mm = (float(pw_us) - PW_OFFSET_US) / PW_US_PER_MM;
  if (mm < 0) mm = 0;
  return mm;
}

// ======= Read distance in mm with optional averaging =======
float readDistanceMM(DistanceDir dir, uint8_t samples) {
  const int pin = distanceSensorPin(dir);
  float vals[7];
  samples = constrain(samples, 1, 7);

  uint8_t got = 0;
  for (uint8_t i = 0; i < samples; i++) {
    unsigned long pw = readPulseUS(pin);
    float mm = pulseToMM(pw);
    if (mm >= 0) vals[got++] = mm;
    delayMicroseconds(500);
  }
  if (got == 0) return -1.0f;

  if (got == 3) {
    float a = vals[0], b = vals[1], c = vals[2];
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
  } else {
    float sum = 0, mn = vals[0], mx = vals[0];
    for (uint8_t i = 0; i < got; i++) {
      sum += vals[i];
      if (vals[i] < mn) mn = vals[i];
      if (vals[i] > mx) mx = vals[i];
    }
    if (got >= 5) return (sum - mn - mx) / float(got - 2);
    return sum / float(got);
  }
}


// -----------------------------------------------------------
//                Cube Following / Avoidance Functions
// -----------------------------------------------------------
void pass_cube(char cube_direction) {
  double angle_addition = (cube_direction == 'R') ? 0 : 5;

  read_gyro_data();
  // translate 'R' → +1, 'L' → -1
  cube_last = (cube_direction == 'R') ? 1 : -1;

  double start_angle = gz;
  move_until_angle(robot_speed, start_angle - cube_last * (AVOIDANCE_ANGLE + angle_addition));
  move_cm_gyro(12, robot_speed, start_angle - cube_last * (AVOIDANCE_ANGLE + angle_addition));

  last_cube_time = millis();
  currentState = AFTER_CUBE;
  flush_messages();
}

// -----------------------------------------------------------
//                           Functions
// -----------------------------------------------------------
void custom_delay(long long delay_time) {
  long long start_time = millis();
  while (millis() - start_time < delay_time) {
    flush_messages();
  }
}

double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void blink_led(int led_pin, int time) {
  digitalWrite(led_pin, HIGH);
  custom_delay(time);
  digitalWrite(led_pin, LOW);
  custom_delay(time);
  digitalWrite(led_pin, LOW);
  custom_delay(time);
  digitalWrite(led_pin, LOW);
  custom_delay(time);
  digitalWrite(led_pin, LOW);
  custom_delay(time);
}

void execute_command(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  //––– 1) Special “S…” steering override for RUN_MODE=1 –––––––––––––––
  if (cmd.startsWith("S") && RUN_MODE == 1) {
    desiredSteering = cmd.substring(1).toFloat();
    currentState = FOLLOW_CUBE;

    float val = desiredSteering;
    follow_cube_angle = val;
    last_follow_cube = millis();

    if (debug) {
      Serial.print("FOLLOW_CUBE angle → ");
      Serial.print(follow_cube_angle);
      Serial.print("      Steer-override → ");
      Serial.println(desiredSteering);
    }
    return;
  }

  //––– 2) Map color words to single-char code, set turn_direction on first BLUE/ORANGE –––
  char c = cmd.charAt(0);
  if (cmd.indexOf("BLACK") != -1) {
    c = 'B';
  } else if (cmd.indexOf("BLUE") != -1) {
    c = 'L';
    lastLineDetectedTime = millis();
    if (turn_direction == 0) turn_direction = -1;
  } else if (cmd.indexOf("ORANGE") != -1) {
    c = 'O';
    lastLineDetectedTime = millis();
    if (turn_direction == 0) turn_direction = 1;
  } else if (cmd.indexOf("RED") != -1) {
    c = 'R';
  } else if (cmd.indexOf("GREEN") != -1) {
    c = 'G';
  } else if (cmd.indexOf("PINK") != -1) {
    c = 'P';
  }

  //––– 3) BLACK-line detection → 90° turn in PID––––––––––––––
  //if (currentState == FOLLOW_CUBE && (c == 'L' || c == 'O')) current_angle_gyro += turn_direction * 90;
  if ((c == 'B' || c == 'L' || c == 'O') && turn_direction != 0) { 
    if (millis() - lastTurnTime > delayTurns) {
      turn_count++;
      if (debug) Serial.println("Received 'BLACK' command. Turning 90°...");
      if (turn_count == 12 && RUN_MODE == 1) {
        move_straight_on_gyro(robot_speed, 720);
        //move_straight_on_gyro_till_dist(robot_speed, 26, FRONT_ULTRASONIC_PIN);
        current_angle_gyro += turn_direction * 90;

        move_straight_on_gyro(robot_speed, 2150);
        double err = current_angle_gyro - gz - turn_direction * 150;
        pid_error = (err)*kp + (pid_error - pid_last_error) * kd;
        pid_last_error = pid_error;
        steer(pid_error);
        
        move(65);
        delay(650);
        move(0);
        delay(20000);
      } else {
        if (abs(current_angle_gyro - gz) < 10 && currentState != AFTER_CUBE) {
          if (RUN_MODE == 1)
            move_cm_gyro(10, robot_speed, current_angle_gyro);  // 15
          else
            move_straight_on_gyro(robot_speed, 140);
        } else if (currentState == AFTER_CUBE) {
          if (-cube_last == turn_direction) {
            move_until_angle(robot_speed, current_angle_gyro + turn_direction * 30);
          } else {
            move_until_angle(robot_speed, current_angle_gyro + turn_direction * 30);
            move_cm_gyro(5, robot_speed, current_angle_gyro + turn_direction * 30);
          }
          currentState = PID;
        }
      }

      current_angle_gyro += turn_direction * 90;

      //move_until_angle_max(robot_speed, current_angle_gyro - 10);
      //move_straight_on_gyro(-robot_speed, 1100);

      delayTurns = 2500;  // 1000 
      lastTurnTime = millis();
      lastLineDetectedTime = 0;
    }
  }


  //––– 4) Cube-avoidance: see a cube up close (R/G) –––––––––––––––––––––
  if ((c == 'R' || c == 'G')) {
    // R means avoid left, G avoid right
    cube_avoid_direction = (c == 'R' ? 'L' : 'R');
    currentState = AVOID_CUBE;
    last_cube_time = millis();
    if (debug) {
      Serial.print("AVOID_CUBE dir → ");
      Serial.println(cube_avoid_direction);
    }
    return;
  }

  //––– 5) All other incoming commands (P, O, L, etc.)—stay in your current state or ignore –––
  if (debugcam) {
    Serial.print("Ignored cmd → ");
    Serial.println(cmd);
  }
}