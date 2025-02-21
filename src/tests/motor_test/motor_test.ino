#define PWMA 11  // PWM pin for motor speed control
#define AIN1 7   // Motor direction pin 1
#define AIN2 8   // Motor direction pin 2
#define STBY 10  // Standby pin

#define PWM_CHANNEL 1   // ESP32 PWM channel (0-15)
#define PWM_FREQ 1000   // PWM frequency in Hz
#define PWM_RESOLUTION 8  // PWM resolution (8-bit: 0-255)

void setup() {
  // Initialize motor control pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable the motor driver by setting STBY (standby) pin HIGH
  digitalWrite(STBY, HIGH);

  // Configure LEDC PWM for motor speed control
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL);
}

// Move motor function using PWM
void move(int speed) {
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;  // Convert negative speed to positive
  }

  ledcWrite(PWM_CHANNEL, speed);  // Set motor speed using LEDC PWM
}

void loop() {
  move(255);  // Move forward at full speed
  delay(3000);   // Continue for 3 seconds

  move(0);    // Stop the motor
  delay(1000);   // Pause for 1 second

  move(-255); // Move backward at full speed
  delay(3000);   // Continue for 3 seconds

  move(0);    // Stop the motor
  delay(1000);   // Pause for 1 second
}
