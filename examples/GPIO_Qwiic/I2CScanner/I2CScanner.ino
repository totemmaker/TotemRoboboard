#include <RoboBoard.h>
/*
  RoboBoard example to scan (selected) GPIO pins for connected I2C devices
  On RoboBoard X3 connect SDA : 33, SCL : 32
  On RoboBoard X4 connect SDA : GPIOA, SCL : GPIOB
   ______                   _____
  |      |       SDA       |     |
  | IO33 | --------------- | SDA |
  | IO32 | --------------- | SCL |
  |______|       SCL       |_____|
   X3 / X4                 I2C device
*/
// Include I2C library
#include <Wire.h>
// Initialize program
void setup() {
  // Start Serial Monitor communication at 115200 speed
  Serial.begin(115200);
  // Initialize RoboBoard
  Board.begin();
  // Abort if board is RoboBoard X4 v1.0
  if (Board.isRoboBoardX4(10)) { while (1) { Serial.println("RoboBoard X4 v1.0 does not support GPIO! See X4_v10 example"); delay(500); } }
  // Initialize I2C library (100kHz speed)
  if (Board.isRoboBoardX3()) { // RoboBoard X3
    Wire1.begin(33, 32); // Select 33 as SDA, 32 as SCL
  }
  else { // RoboBoard X4
    Wire1.begin(GPIOA, GPIOB); // Select GPIOA as SDA, GPIOB as SCL
  }
  // Change I2C frequency (speed) if required
  // Wire1.setClock(400000); // set to 400kHz
  // SDA, SCL are definitons of pins, connected to Qwiic port
  // Recommendation:
  // For Qwiic port use "Wire"
  // For GPIO pins use "Wire1"
}
// Loop program
void loop() {
  // Print header text
  Serial.println();
  Serial.println("Detected I2C modules:");
  // Variable to check if any module is detected
  bool isAnythingDetected = false;
  // Loop all available I2C addresses
  int address = 0x01;
  int endAddress = 0x7D;
  // Loop all I2C devices
  for (; address <= endAddress; address++) {
    // Start I2C transmission to selected address
    Wire1.beginTransmission(address);
    // Get result of transmission
    int error = Wire1.endTransmission();
    // Check if there was no error during transmission
    if (error == 0) {
      // Print detected address
      Serial.print("I2C address: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      // IMU sensor is connected to Qwiic
      // I2C line. Print sensor name
      if (IMU.isI2CAddr(address)) {
        Serial.print(" (");
        Serial.print(IMU.getName());
        Serial.print(")");
      }
      Serial.println();
      // Mark that we found something
      isAnythingDetected = true;
    }
  }
  // Print message if nothing was found
  if (!isAnythingDetected)
    Serial.println("None");
  // Loop every 3 seconds
  delay(3000);
}