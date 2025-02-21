#include <ESP32Servo.h>

#define SteeringServoPin 2

Servo steeringServo;

void setup() {
  Serial.begin(115200); 
  steeringServo.attach(SteeringServoPin); 
  Serial.println("Servo Test Started...");
}

void loop() {
  steeringServo.write(120);
  delay(500);
  steeringServo.write(80);   
  delay(500);
  steeringServo.write(30);
  delay(500);
}