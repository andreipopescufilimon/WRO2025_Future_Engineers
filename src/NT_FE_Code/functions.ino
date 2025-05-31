#define AVOIDANCE_ANGLE 37  // how far to pivot around the cube

#define EXIT_MOVE_TIME 300    // Duration (ms) for the aggressive exit move
#define RETURN_MOVE_TIME 500  // Duration (ms) for the aggressive return move
#define CORRECTION_ANGLE 30   // how much extra to hold when driving past cube

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
//                Cube Following / Avoidance Functions
// -----------------------------------------------------------
void pass_cube(char cube_direction) {
  read_gyro_data();
  // translate 'R' → +1, 'L' → -1
  int cube_last = (cube_direction == 'R') ? 1 : -1;

  // 1) pivot away from cube by AVOIDANCE_ANGLE
  double start_angle = gz;
  move_until_angle(robot_speed, start_angle - cube_last * AVOIDANCE_ANGLE);

  // 2) drive forward ~4 cm while holding a slight offset so you clear the cube
  move_cm_gyro(8, robot_speed, start_angle - cube_last * AVOIDANCE_ANGLE);

  last_cube_time = millis();
  currentState = AFTER_CUBE;

  while (millis() - last_cube_time > 1500) {
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
  } else {
    return;  // we don’t change state here—loop() will handle the next drive step
  }

  //––– 3) BLACK-line detection → 90° turn in PID––––––––––––––
  if (currentState == PID) {
    if (c == 'B' || (millis() - lastLineDetectedTime > 1800 && lastLineDetectedTime > 0)) {
      if (millis() - lastTurnTime < 1000) {
        if (debug) Serial.println("Ignoring repeated 'B' command due to cooldown.");
        return;
      }
      if (debug) Serial.println("Received 'BLACK' command. Turning 90°...");
      current_angle_gyro += turn_direction * 90;

      turn_count++;
      lastTurnTime = millis();
      lastLineDetectedTime = 0;
      return;
    }
  }

  //––– 4) Cube-avoidance: see a cube up close (R/G) –––––––––––––––––––––
  if ((c == 'R' || c == 'G') && millis() - last_cube_time >= AVOIDANCE_DRIVE_TIME) {
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