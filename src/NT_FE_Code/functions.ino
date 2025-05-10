#define AVOIDANCE_ANGLE 40  // how far to pivot around the cube

#define EXIT_MOVE_TIME 300         // Duration (ms) for the aggressive exit move
#define RETURN_MOVE_TIME 500       // Duration (ms) for the aggressive return move
#define CORRECTION_ANGLE 30        // how much extra to hold when driving past cube
unsigned long last_cube_time = 0;  // remember when we last avoided one

// -----------------------------------------------------------
//                Cube Following / Avoidance Functions
// -----------------------------------------------------------
void pass_cube(char cube_direction) {
  read_gyro_data();
  // translate 'R' → +1, 'L' → -1
  int cube_last = (cube_direction == 'R') ? 1 : -1;

  // 1) pivot away from cube
  double start_angle = gz;
  move_until_angle(robot_speed, start_angle - cube_last * (AVOIDANCE_ANGLE));

  // 2) drive forward ~12 cm while holding a slight offset so you clear the cube
  move_cm_gyro(4, robot_speed, gz + cube_last * CORRECTION_ANGLE);

  last_cube_time = millis();
  currentState = AFTER_CUBE;
}


// -----------------------------------------------------------
//                           Functions
// -----------------------------------------------------------
void custom_delay(long long delay_time) {
  long long start_time = millis();
  while (millis() - start_time < delay_time) {}
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

void execute_command(String command) {
  command.trim();
  if (command.startsWith("S") && RUN_MODE == 1) {
    desiredSteering = command.substring(1).toFloat();
    currentState = FOLLOW_CUBE;
    Serial.println(desiredSteering);
    return;
  }

  if (command.indexOf("F") != -1) {
    return;
  }

  char cmd = command.charAt(0);
  switch (currentState) {
    case DEFAULT_CASE:
      {
        if (cmd == 'L') {
          if (turn_direction == 0) {
            turn_direction = -1;
            lastLineDetectedTime = millis();
          }
        } else if (cmd == 'O') {
          if (turn_direction == 0) {
            turn_direction = 1;
            lastLineDetectedTime = millis();
          }
        }
        if (cmd == 'B' || (millis() - lastLineDetectedTime > 1200 && lastLineDetectedTime > 0)) {
          if (millis() - lastTurnTime < 1000) {
            if (debug) Serial.println("Ignoring repeated 'B' command due to cooldown.");
            return;
          }
          if (debug) Serial.println("Received 'BLACK' command. Turning 90°...");
          current_angle_gyro += turn_direction * 90;

          turn_count++;
          lastTurnTime = millis();
          lastLineDetectedTime = 0;
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
        currentState = DEFAULT_CASE;
        break;
      }

    case AVOID_CUBE:
      {
        pass_cube(cube_avoid_direction);
        break;
      }

    case FOLLOW_CUBE:
      {
        steer(desiredSteering);
        move(robot_speed);
        if (cmd == 'R') {
          cube_avoid_direction = 'L';
          currentState = AVOID_CUBE;
        } else if (cmd == 'G') {
          cube_avoid_direction = 'R';
          currentState = AVOID_CUBE;
        }

        if (cmd == 'L') {
          if (turn_direction == 0) {
            turn_direction = -1;
            lastLineDetectedTime = millis();
          }
        } else if (cmd == 'O') {
          if (turn_direction == 0) {
            turn_direction = 1;
            lastLineDetectedTime = millis();
          }
        }

        if (cmd == 'B' || (millis() - lastLineDetectedTime > 1200 && lastLineDetectedTime > 0)) {
          if (millis() - lastTurnTime < 1000) {
            if (debug) Serial.println("Ignoring repeated 'B' command due to cooldown.");
            return;
          }
          if (debug) Serial.println("Received 'BLACK' command. Turning 90°...");
          current_angle_gyro += turn_direction * 90;

          turn_count++;
          lastTurnTime = millis();
          lastLineDetectedTime = 0;
        }
        break;
      }

    case AFTER_CUBE:
      {
        last_cube_time = millis();
        while (millis() - last_cube_time > 500) {
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
        currentState = DEFAULT_CASE;
        cameraSerial.flush();
        break;
      }

    default:
      {
        break;
      }
  }
}