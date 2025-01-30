#include <RoboBoard.h>
/*
  Board:  [X3] RoboBoard X3
  RoboBoard example to interact with GPIO pins.
*/
// Initialize program
void setup() {
  // Initialize RoboBoard
  Board.begin();
  // Check board
  if (!Board.isRoboBoardX3()) { while (1) { Serial.println("Example is for RoboBoard X3!"); delay(500); } }
  // Use standard Arduino API to interact with GPIO pins
  pinMode(26, INPUT_PULLDOWN);  // Set pin 26 to INPUT with pulling to LOW
  pinMode(32, OUTPUT); // Set pin 32 to OUTPUT
  pinMode(33, OUTPUT); // Set pin 33 to OUTPUT
}
// Loop program
void loop() {
  digitalWrite(32, HIGH); // Set pin 32 to HIGH (3.3 Volts -> VCC)
  digitalWrite(33, LOW); // Set pin 33 to LOW (0 Volts -> GND)
  delay(1000); // Wait 1 second
  digitalWrite(32, LOW); // Set pin 32 to LOW (0 Volts -> GND)
  digitalWrite(33, HIGH); // Set pin 33 to HIGH (3.3 Volts -> VCC)
  delay(1000); // Wait 1 second
  if (digitalRead(26)) { // Turn green color if pin 26 == HIGH
    RGB.color(Color::Green);
  }
  else { // Turn red color if pin 26 == LOW
    RGB.color(Color::Red);
  }
}