#include <RoboBoard.h>
/*
  CAN bus monitor example. Print received packets to Serial Monitor.

  Documentation: https://docs.totemmaker.net/roboboard/api/can

  Note: Works with RoboBoard X4 only!
*/

// #define DISPLAY_TIMESTAMP
// #define LISTEN_MODE

void setup() {
  Serial.begin(115200);
  // Initialize RoboBoard
  Board.begin();
  // Check board
  if (!Board.isRoboBoardX4()) { while (1) { Serial.println("Only RoboBoard X4 has CANBUS!"); delay(500); } }
#ifdef LISTEN_MODE
  CAN.beginListen(500);
#else
  CAN.begin(500); // 25, 50, 100, 125, 250, 500, 800, 1000
#endif
}
// Loop program
void loop() {
  if (CAN.readPacketWait()) {
    auto packet = CAN.getPacket();
#ifdef DISPLAY_TIMESTAMP
    Serial.printf("\n%08d %s %8X [%d]",
    micros(),
#else
    Serial.printf("\n%s %8X [%d]",
#endif
    packet.ext ? "(EXT) " : "(STD) ",
    packet.id,
    packet.len);
    if (packet.rtr) {
      Serial.printf(" (RTR)");
    }
    else {
      for (int i=0; i<packet.len; i++) {
        Serial.printf(" %02X", packet.data[i]);
      }
    }
  }
}
