#include <RoboBoard.h>
/*
  RoboBoard example to interact with external LED.
*/
IOLED led(33); // RoboBoard X3
// IOLED led(GPIOA); // RoboBoard X4
// Initialize program
void setup() {
  // Initialize RoboBoard
  Board.begin();
}
// Loop program
void loop() {
  // Turn on
  led.on();
  delay(1000);
  // Turn off
  led.off();
  delay(1000);
  // Blink 5 times
  led.blink(5);
  led.wait(); // Wait until blink stops
}