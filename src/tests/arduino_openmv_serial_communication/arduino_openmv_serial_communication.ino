#include <SoftwareSerial.h>

// Define RX and TX pins for software serial
SoftwareSerial mySerial(0, 1);  // RX, TX (Change pins if necessary)

String receivedMessage = "";  // Store incoming message

void setup() {
  Serial.begin(9600);   // Serial Monitor (PC Debugging)
  mySerial.begin(19200);  // UART Communication with OpenMV
}

void loop() {
  while (mySerial.available() > 0) {      // If characters are available
    char receivedChar = mySerial.read();  // Read one character at a time

    if (receivedChar == '\n') {  // If we reach the end of a message
      Serial.print("Received from OpenMV: ");
      Serial.println(receivedMessage);  // Print the full message
      receivedMessage = "";             // Reset message buffer
    } else {
      receivedMessage += receivedChar;  // Append character to message buffer
    }
  }
}
