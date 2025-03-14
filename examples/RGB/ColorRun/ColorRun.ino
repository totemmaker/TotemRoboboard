#include <RoboBoard.h>
/*
  RoboBoard example to display random color "running" animation
*/
// Initialize program
void setup() {
  // Initialize RoboBoard
  Board.begin();
}
// List to hold current colors
int colorList[4];
// Loop program
void loop() {
  // Shift all colors to left
  for (int i=0; i<3; i++) {
    colorList[i] = colorList[i+1];
  }
  // Generate new random color
  colorList[3] = Color::random();
  // Output color list to LEDs
  for (int i=0; i<4; i++) {
    RGB[i].color(colorList[i]);
  }
  delay(120);
}
