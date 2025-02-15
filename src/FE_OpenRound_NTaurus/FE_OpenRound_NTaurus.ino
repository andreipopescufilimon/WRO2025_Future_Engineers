String receivedMessage = "";  // Store incoming message

void setup() {
  Serial.begin(9600);   // Serial Monitor (PC Debugging)
  Serial0.begin(19200);  // UART Communication with OpenMV (P4 TX → D0 RX)
}

void loop() {
  while (Serial0.available() > 0) {      // If characters are available
    char receivedChar = Serial0.read();  // Read one character at a time

    if (receivedChar == '\n') {  // If we reach the end of a message
      Serial.print("Received from OpenMV: ");
      Serial.println(receivedMessage);  // Print the full message
      receivedMessage = "";             // Reset message buffer
    } else {
      receivedMessage += receivedChar;  // Append character to message buffer
    }
  }
}
