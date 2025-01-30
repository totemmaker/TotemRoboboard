#include <RoboBoard.h>
#include <TotemModule22.h>
/*
  Module:  [22] Totem Environment sensor module
  RoboBoard example to control status LED
*/
// Initialize Environment sensor module
TotemModule22 sensor;
// Initialize program
void setup() {
  // Initialize RoboBoard
  Board.begin();
  // Check board
  if (!Board.isRoboBoardX4()) { while (1) { Serial.println("Example is for RoboBoard X4!"); delay(500); } }
}
// Loop program
void loop() {
  // Play some LED animations
  /*
    Pulse blinking
  */
  for (int t=10; t<200; t+=10) {
    sensor.led.toggle();
    delay(t);
  }
  for (int t=200; t>=10; t-=10) {
    sensor.led.toggle();
    delay(t);
  }
}