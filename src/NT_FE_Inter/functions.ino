#define AVOIDANCE_ANGLE 50  // how far to pivot around the cube
int delayTurns = 2000;

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
float readDistanceMM(DistanceDir dir) {
  int pin;
  switch (dir) {
    case FRONT_DIR: pin = PWM_DIST_FRONT; break;  // 300 cm
    case BACK_DIR: pin = PWM_DIST_BACK; break;    // 130 cm
    case LEFT_DIR: pin = PWM_DIST_LEFT; break;    // 50 cm
    case RIGHT_DIR: pin = PWM_DIST_RIGHT; break;  // 50 cm
    default: return -1;
  }

  pinMode(pin, INPUT);  // if your sensor is open-drain, use INPUT_PULLUP

  // Re-sync so we start at the beginning of the next cycle.
  pulseInLong(pin, LOW, 100000);                     // wait up to 100 ms for LOW
  unsigned long t = pulseInLong(pin, HIGH, 100000);  // measure HIGH width

  if (t == 0) return -1;    // timeout = no reading
  if (t > 1850) return -1;  // Pololu “no detect” window

  float mm;
  if (dir == FRONT_DIR) mm = (t - 1000) * 4.0f;      // 300 cm
  else if (dir == BACK_DIR) mm = (t - 1000) * 2.0f;  // 130 cm
  else mm = (t - 1000) * 0.75f;                      // 50 cm (left/right)

  if (mm < 0) mm = 0;
  return mm;
}


void debug_distance_infinite(DistanceDir dir) {
  while (true) {
    float d_mm = readDistanceMM(dir);

    if (dir == FRONT_DIR) {
      Serial.print("Front distance: ");
    } else if (dir == BACK_DIR) {
      Serial.print("Back distance: ");
    } else {
      Serial.print("Other distance: ");
    }

    if (d_mm < 0) {
      Serial.println("No reading");
    } else {
      Serial.print(d_mm);
      Serial.println(" mm");
    }
  }
}

// -----------------------------------------------------------
//                Cube Following / Avoidance Functions
// -----------------------------------------------------------
void pass_cube(char cube_direction) {
  double angle_addition = (cube_direction == 'R') ? 5 : 5;

  read_gyro_data();
  // translate 'R' → +1, 'L' → -1
  cube_last = (cube_direction == 'R') ? 1 : -1;

  double start_angle = gz;
  move_until_angle(robot_speed, start_angle - cube_last * (AVOIDANCE_ANGLE + angle_addition));
  move_cm_gyro(7, robot_speed, start_angle - cube_last * (AVOIDANCE_ANGLE + angle_addition));

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
  if ((c == 'L' || c == 'O') && turn_direction != 0) {  // c == 'B' ||
    if (millis() - lastTurnTime > delayTurns) {
      turn_count++;
      if (debug) Serial.println("Received 'BLACK' command. Turning 90°...");
      if (turn_count == 12 && RUN_MODE == 1) {
        move_straight_on_gyro(robot_speed, 500);
        if (turn_direction == -1) {
          current_angle_gyro -= 5;
          move_to_distance(FRONT_DIR, 420.0f, 10.0f, 55, current_angle_gyro);
          current_angle_gyro += turn_direction * 90;

          move_until_angle_max(park_speed + 5, current_angle_gyro);
          move_straight_on_gyro(robot_speed, 300);
          move_to_distance(BACK_DIR, 300.0f, 10.0f, -park_speed, current_angle_gyro);
          move_straight_on_gyro(robot_speed, 1400);
          move_gyro_until_side_detect_cycle(RIGHT_DIR, park_speed, current_angle_gyro);
          move_cm_gyro(13, park_speed, current_angle_gyro);

          // sensu de stanga
          move_until_angle_max(-park_speed, (current_angle_gyro) - 75);
          move_until_angle_max(-park_speed, (current_angle_gyro) - 5);
          move_until_angle_max(park_speed, (current_angle_gyro) + 5);
        } else {
          current_angle_gyro += 5;
          move_to_distance(FRONT_DIR, 420.0f, 10.0f, 55, current_angle_gyro);
          current_angle_gyro += turn_direction * 90;

          move_until_angle_max(park_speed + 5, current_angle_gyro);
          move_straight_on_gyro(robot_speed, 300);
          move_to_distance(BACK_DIR, 300.0f, 10.0f, -park_speed, current_angle_gyro);
          move_gyro_until_side_detect_cycle(LEFT_DIR, park_speed, current_angle_gyro);
          move_cm_gyro(9, park_speed, current_angle_gyro);

          move_until_angle_max(-park_speed, (current_angle_gyro) + 75);
          move_until_angle_max(-park_speed, (current_angle_gyro) + 5);
          move_until_angle_max(park_speed, (current_angle_gyro) - 5);
        }

        steeringServo.write(STEERING_CENTER);
        delay(300);

        move(0);
        delay(20000);
      } else {
        if (abs(current_angle_gyro - gz) < 10 && currentState != AFTER_CUBE) {
          if (RUN_MODE == 1)
            move_cm_gyro(15, robot_speed, current_angle_gyro);  // 15
          else
            move_straight_on_gyro(robot_speed, 150);
        } else if (currentState == AFTER_CUBE) {  
          if (-cube_last == turn_direction) {
            move_until_angle(robot_speed, current_angle_gyro + turn_direction);  // *30
          } else {
            move_until_angle(robot_speed, current_angle_gyro + turn_direction * 30);
            move_cm_gyro(4, robot_speed, current_angle_gyro + turn_direction * 30);
          }
          currentState = PID;
        }
      }

      current_angle_gyro += turn_direction * 90;

      //move_until_angle_max(robot_speed, current_angle_gyro - 10);
      //move_straight_on_gyro(-robot_speed, 1100);

      delayTurns = 2000;  // 1000 // 2000
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