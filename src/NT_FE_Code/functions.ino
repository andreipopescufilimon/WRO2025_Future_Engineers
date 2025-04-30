// -----------------------------------------------------------
//                Cube Following / Avoidance Functions
// -----------------------------------------------------------
void follow_cube(float steeringCorrection) {
  int newSteering = STEERING_CENTER - (int)steeringCorrection;
  newSteering = constrain(newSteering, STEERING_LEFT, STEERING_RIGHT);
  steeringServo.write(newSteering);
  if (debug) {
    Serial.print("Following cube. Steering correction: ");
    Serial.println(steeringCorrection);
  }
  move(robot_speed);
}

void pass_cube(char cube_direction) {
  if (cube_direction == 'R') {
    if (debug) {
      Serial.println("Pass cube: Red avoidance sequence: initial turn right then exit left.");
    }
    steeringServo.write(STEERING_AVOID_RIGHT);
    move(robot_speed);
    delay(EXIT_MOVE_TIME);

    steeringServo.write(STEERING_LEFT);
    move(robot_speed);
    delay(RETURN_MOVE_TIME);

    steeringServo.write(STEERING_CENTER);
  } else if (cube_direction == 'L') {
    if (debug) {
      Serial.println("Pass cube: Green avoidance sequence: initial turn left then exit right.");
    }
    steeringServo.write(STEERING_AVOID_LEFT);
    move(robot_speed);
    delay(EXIT_MOVE_TIME);

    steeringServo.write(STEERING_RIGHT);
    move(robot_speed);
    delay(RETURN_MOVE_TIME);

    steeringServo.write(STEERING_CENTER);
  }
  turn_direction = 0;
  currentState = DEFAULT_CASE;
}

// -----------------------------------------------------------
//                           Functions
// -----------------------------------------------------------
void custom_delay(long long delay_time) {
  long long start_time = millis();
  while (millis() - start_time < delay_time) {}
}

void blink_led(int led_pin, int time) {
  digitalWrite(led_pin, HIGH);
  custom_delay(time);
  digitalWrite(led_pin, LOW);
  custom_delay(time);
}

void execute_command(String command) {
  command.trim();
  if (command.startsWith("S") && RUN_MODE == 1) {
    desiredSteering = command.substring(1).toInt();
    currentState = FOLLOW_CUBE;
    return;
  }
  if (command.indexOf("F") != -1) {
    robot_speed = follow_speed;
    return;
  }

  char cmd = command.charAt(0);
  switch (currentState) {
    case DEFAULT_CASE:{
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
      if (cmd == 'B' || (millis() - lastLineDetectedTime > 500 && lastLineDetectedTime > 0)) {
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

    case AVOID_CUBE:{
      pass_cube(cube_avoid_direction);
      break;
    }

    case FOLLOW_CUBE:{
      steeringServo.write(desiredSteering);
      move(robot_speed);
      if (cmd == 'R') {
        cube_avoid_direction = 'L';
        currentState = AVOID_CUBE;
      } else if (cmd == 'G') {
        cube_avoid_direction = 'R';
        currentState = AVOID_CUBE;
      }
      break;
    }

    case AFTER_CUBE:{
      read_gyro_data();
      double error = current_angle_gyro - gx;
      pid_error = (error)*kp + (pid_error - pid_last_error) * kd;
      pid_last_error = pid_error;
      steer(pid_error);
      move(robot_speed);
      currentState = DEFAULT_CASE;
      break;
    }

    default: {
      read_gyro_data();
      double error = current_angle_gyro - gx;
      pid_error = (error)*kp + (pid_error - pid_last_error) * kd;
      pid_last_error = pid_error;
      steer(pid_error);
      move(robot_speed);
      currentState = DEFAULT_CASE;
      break;
    }
  }
}