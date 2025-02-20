#include <ESP32Servo.h>

#define SteeringServoPin 3 

Servo steeringServo;

void setup() {
  Serial.begin(115200); 
  steeringServo.attach(SteeringServoPin, 500, 2500); 
  Serial.println("Servo Test Started...");
}

void loop() {
  steeringServo.write(130);
  delay(500);
  steeringServo.write(80);   
  delay(500);
  steeringServo.write(30);
  delay(500);
}