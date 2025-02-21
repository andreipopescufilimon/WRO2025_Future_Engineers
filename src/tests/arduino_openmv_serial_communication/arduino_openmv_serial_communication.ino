char command = '0';
int turn_direction = 0;
String receivedMessage = "";  // Store incoming message

void setup() {
  Serial.begin(9600);    // Serial Monitor (PC Debugging)
  Serial0.begin(19200);  // UART Communication with OpenMV
}

void loop() {
  command = '0';
  while (Serial0.available() > 0) {      // If characters are available
    char receivedChar = Serial0.read();  // Read one character at a time

    if (receivedChar == '\n') {  // If we reach the end of a message
      //Serial.print("Received from OpenMV: ");
      //Serial.println(receivedMessage);  // Print the full message

      receivedMessage.toUpperCase();
      if (receivedMessage.indexOf("BLACK") != -1) {
        command = 'B';
      } else if (receivedMessage.indexOf("BLUE") != -1) {
        command = 'L';
        if (turn_direction == 0) turn_direction = -1;
      } else if (receivedMessage.indexOf("ORANGE") != -1) {
        command = 'O';
        if (turn_direction == 0) turn_direction = 1;
      } else if (receivedMessage.indexOf("RED") != -1) {
        command = 'R';
      } else if (receivedMessage.indexOf("GREEN") != -1) {
        command = 'G';
      } else if (receivedMessage.indexOf("PINK") != -1) {
        command = 'P';
      }
      Serial.print("Current command: ");
      Serial.println(command);

      receivedMessage = "";  // Reset message buffer
    } else {
      receivedMessage += receivedChar;  // Append character to message buffer
    }
  }
}
