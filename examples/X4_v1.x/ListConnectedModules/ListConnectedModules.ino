#include <RoboBoard.h>
#include <TotemModule11.h>
/*
  RoboBoard X4 example to find connected TotemBUS modules
*/
// Initialize program
void setup() {
  // Start Serial Monitor communication at 115200 speed
  Serial.begin(115200);
  // Initialize RoboBoard
  Board.begin();
  // Check board
  if (!Board.isRoboBoardX4()) { while (1) { Serial.println("Example is for RoboBoard X4!"); delay(500); } }
}
// Loop program
void loop() {
  // Start list of modules
  Serial.println("Connected module:");
  // Ping modules every 1s to receive their response
  // if (TotemX4Module::find(11)) { // Search for distance sensor module
  // if (TotemX4Module::find(11, 9724)) { // Search for distance sensor module with serial 9724
  if (TotemX4Module::find()) {
    // Print module information
    Serial.print("number: ");
    Serial.print(TotemX4Module::foundNumber);
    Serial.print(", serial: ");
    Serial.println(TotemX4Module::foundSerial);
  }
  else {
    Serial.println("No modules found");
  }
  // Wait 1s
  delay(1000);
}