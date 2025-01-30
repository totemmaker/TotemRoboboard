#include <RoboBoard.h>
#include <Wire.h>
/*
  RoboBoard example to view gyroscope measurements on
  Arduino IDE Serial Plotter.
  1. Upload this example
  2. Open Tools -> Serial Plotter
  3. Select 115200 baud rate
*/
// Initialize program
void setup() {
  // Start Serial communication at 115200 speed
  Serial.begin(115200);
  // Initialize RoboBoard
  Board.begin();
  // Start I2C. IMU sensor internally connected to Qwiic port
  Wire.begin();
  Wire.setClock(400000); // Set 400kHz speed
}
// Loop program
void loop() {
  // Read measurements from the sensor and store to "result" variable
  auto result = IMU.read();
  // Print measurements
  Serial.printf("Min:-2000,");
  Serial.printf("GyroX:%.f,", result.getX_dps());
  Serial.printf("GyroY:%.f,", result.getY_dps());
  Serial.printf("GyroZ:%.f,", result.getZ_dps());
  Serial.printf("Max:2000\n");
  delay(25); // Wait 25 milliseconds
}
