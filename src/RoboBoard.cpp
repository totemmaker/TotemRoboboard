#include <Wire.h>

void RoboBoard_initialize_wire(int sda0, int scl0, int sda1, int scl1) {
    Wire.setPins(sda0, scl0);
    if (sda1 != -1) Wire1.setPins(sda1, scl1);
}
