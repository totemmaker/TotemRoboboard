#include <RoboBoard.h>
#include <Wire.h>
/*
  RoboBoard example to read integrated RoboBoard IMU sensor.
  Prints Accelerometer and Gyroscope data.
*/
// Initialize program
void setup() {
  // Start Serial Monitor communication at 115200 speed
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
  // Formatting: https://www.cplusplus.com/reference/cstdio/printf/
  Serial.printf("Acc: X:% 1.2f Y:% 1.2f Z:% 1.2f | Gyro: X:% 4.f Y:% 4.f Z:% 4.f | Orient: X:% 3.0f, Y:% 3.0f | Temp: %.1fC\n", 
      result.getX_G(), // X axis accelerometer gyroscopic force
      result.getY_G(), // Y axis accelerometer gyroscopic force
      result.getZ_G(), // Z axis accelerometer gyroscopic force
      result.getX_dps(), // X axis gyroscope rotation degrees per second
      result.getY_dps(), // Y axis gyroscope rotation degrees per second
      result.getZ_dps(), // Z axis gyroscope rotation degrees per second
      result.getOrientX(), // accelerometer based board orientation X axis
      result.getOrientY(), // accelerometer based board orientation Y axis
      result.getTempC() // sensor temperature Celsius
      );
  delay(100); // Wait 100 milliseconds
}
