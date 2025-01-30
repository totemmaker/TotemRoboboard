#include <RoboBoard.h>
/*
  Example of using Button library with button connected to GPIO pin.
  Available RoboBoard X3 pins are 33, 32, 26, SIGA, SIGB, SIGC, SIGD
  Available RoboBoard X4 pins are GPIOA, GPIOB, GPIOC, GPIOD
*/
// Library to read button state located on the board
IOButton button(33); // RoboBoard X3
// IOButton button(GPIOA); // RoboBoard X4
// Initialize program
void setup() {
  Serial.begin(115200);
  // Initialize RoboBoard
  Board.begin();
}
// Loop program
void loop() {
  if (button.isPressedFor(1000)) {
    Serial.println("Press for 1000ms");
  }
  else if (button.isPressed()) { // on button press
    Serial.println("Press");
  }
  else if (button.isReleased()) { // on button release
    Serial.println("Release");
  }
  delay(250);
}
