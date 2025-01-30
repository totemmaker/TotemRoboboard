/*
  RoboBoard.h has predefined control functions like: Board, Button, CAN, RGB, IMU, ...
  Some third-party Arduino libraries may define variables with same name (like "RGB") and it
  will result in compile errors. This example demonstrates how to resolve this.
*/
#define REQUIRE_TOTEM_PREFIX
// Include RoboBoard.h after REQUIRE_TOTEM_PREFIX
#include <RoboBoard.h>
// Now all the functions (Board, CAN, RGB, ...) has to use "totem::" prefix (C++ namespace).
// totem::Board, totem::CAN, totem::RGB, ...
// This allows to define names that conflicts with RoboBoard global functions
class RGBConverter {
public:
    int toHex(uint8_t red, uint8_t green, uint8_t blue) {
        return red << 16 | green << 8 | blue;
    }
};
// We can create object with name "RGB"
RGBConverter RGB;
// Initialize program
void setup() {
  // Initialize RoboBoard
  totem::Board.begin();
  // Custom RGB can be accessed
  // Will error in "reference to 'RGB' is ambiguous" if "Totem Prefix" not selected
  int color = RGB.toHex(0, 255, 0);
  // Totem RoboBoard functions can be accessed under "totem::" namespace
  totem::RGB.color(color);
}
// Loop program
void loop() {

}
