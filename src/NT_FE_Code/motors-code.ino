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
  move(-10);
  delay(20);
  move(0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CHANNEL, 0);
}

// -----------------------------------------------------------
//                    Steering Functions
// -----------------------------------------------------------
void steering_servo_setup() {
  steeringServo.attach(STEERING_SERVO);
  steeringServo.write(STEERING_CENTER);
  delay(200);
}

void steer(int steering_angle) {
  steeringServo.write(steering_angle);
}

void move_straight_on_gyro(double speed, long duration_ms) {
  int sign = speed >= 0 ? 1 : -1;

  unsigned long start_time = millis();
  while (millis() - start_time < duration_ms) {
    read_gyro_data();
    double err = current_angle_gyro - gx;  // try to hold a constant angle
    pid_error = (err)*kp + (pid_error - pid_last_error) * kd;
    pid_last_error = pid_error;

    steer(pid_error * sign);
    move(robot_speed);
    cameraSerial.flush();
  }
  move(0);  // stop  
}