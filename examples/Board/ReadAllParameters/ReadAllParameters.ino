#include <WiFi.h>
#include <Wire.h>
#include <RoboBoard.h>
/*
  Example to read all RoboBoard parameters.
*/
// Initialize program
void setup() {
  // Start Serial Monitor communication at 115200 speed
  Serial.begin(115200);
  // Initialize RoboBoard
  Board.begin();
  // Required by IMU
  Wire.begin();
}
// Loop program
void loop() {
  // printf formatting rules explained in: https://cplusplus.com/reference/cstdio/printf/
  // Print Board configuration
  Serial.printf("\n\n-----Board-----\n");
  Serial.printf("RoboBoard X%d\t%s (revision)\n", Board.getNumber(), Board.getRevisionStr());
  Serial.printf("MAC:\t\t%s\n", WiFi.macAddress().c_str());
  if (Board.isRoboBoardX4()) {
    Serial.printf("Driver:\t\t%s\n", Board.getDriverVersionStr());
  }
  Serial.printf("Software:\t%s\n", Board.getSoftwareVersionStr());
  Serial.printf("Name:\t\t%s\n", Board.getName());
  Serial.printf("Boot color:\t0x%x\n", Board.getColor());
  Serial.printf("USB in:\t\t%s\n", Board.isUSB() ? "Yes" : "No");
  Serial.printf("\n----Settings---\n");
  if (Board.isRoboBoardX3()) {
    Serial.printf("3V3 regulator:\t%s\n", Board.getEnable3V3() ? "Enabled" : "Disabled");
    Serial.printf("Charging mode:\t%s\n", Board.getChargingMode() ? "Enabled" : "Disabled");
  }
  Serial.printf("Status RGB:\t%s\n", Board.getStatusRGB() ? "Enabled" : "Disabled");
  Serial.printf("Status sound:\t%s\n", Board.getStatusSound() ? "Enabled" : "Disabled");
  // Print IMU sensor stats
  Serial.printf("\n------IMU------\n");
  Serial.printf("IMU temp:\t%2.1fC\n", IMU.read().getTempC());
  Serial.printf("IMU sensor:\t%s\n", IMU.getName());
  Serial.printf("I2C address:\t0x%x\n", IMU.getI2CAddr());
  Serial.printf("Accel range:\t%d G\n", IMU.getAccelRange());
  Serial.printf("Gyro range:\t%d dps\n", IMU.getGyroRange());
  // Print battery info
  Serial.printf("\n----Battery----\n");
  Serial.printf("SOC: \t\t%d%%\n", Battery.getSOC());
  if (Board.isRoboBoardX3()) { // getCurrent() and isCharging() only available in RoboBoard X3
    Serial.printf("Voltage:\t%.2fV %s\n", Battery.getVoltage(), Board.isUSB() ? "(USB)" : "");
    if (Board.isRoboBoardX3(30)) // v3.0 does not support current sensing
      Serial.printf("Current:\t(not supported)\n");
    else
      Serial.printf("Current:\t%.2fA\n", Battery.getCurrent());
    Serial.printf("Charging:\t%s\n", Battery.isCharging() ? "Yes" : "No");
  }
  else { // RoboBoard X4
    Serial.printf("Voltage:\t%.2fV\n", Battery.getVoltage());
  }
  // DC settings
  int cnt = 4;
  Serial.printf("\n-------DC------\n");
  Serial.printf("\nPWM Frequency:\t%dHz\n", DC.getFrequency());
  Serial.printf("DC\t\tA\tB\tC\tD");
  Serial.printf("\nEnabled:");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%s", DC[i].getEnable() ? "Yes" : "No"); }
  Serial.printf("\nInvert:\t");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%s", DC[i].getInvert() ? "Yes" : "No"); }
  Serial.printf("\nDecay:\t");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%s", DC[i].getFastDecay() ? "Fast" : "Slow"); }
  Serial.printf("\nAutobrake:");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d%%", DC[i].getAutobrake()); }
  Serial.printf("\nRange:\t");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d:%d", DC[i].getRange().min, DC[i].getRange().max); }
  Serial.printf("\nSpin:\t");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d%%", DC[i].getSpin()); }
  // Servo settings
  Serial.printf("\n\n-----Servo-----\n");
  Serial.printf("\nPWM Period:\t%dus (%dHz)\n", Servo.getPeriod(), 1000000/Servo.getPeriod());
  cnt = Servo.getPortsCount(); // Returns number of servo ports board has
  if (cnt == 2) Serial.printf("Servo\t\tA\tB");
  if (cnt == 3) Serial.printf("Servo\t\tA\tB\tC");
  if (cnt == 4) Serial.printf("Servo\t\tA\tB\tC\tD");
  Serial.printf("\nEnabled:");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%s", Servo[i].getEnable() ? "Yes" : "No"); }
  Serial.printf("\nInvert:\t");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%s", Servo[i].getInvert() ? "Yes" : "No"); }
  Serial.printf("\nMotor angle:");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d", Servo[i].getMotor().angle); }
  Serial.printf("\nMotor usMin:");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d", Servo[i].getMotor().usMin); }
  Serial.printf("\nMotor usMax:");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d", Servo[i].getMotor().usMax); }
  Serial.printf("\nTrim min:");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d", Servo[i].getTrim().min); }
  Serial.printf("\nTrim mid:");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d", Servo[i].getTrim().mid); }
  Serial.printf("\nTrim max:");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d", Servo[i].getTrim().max); }
  Serial.printf("\nRPM:\t");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d", Servo[i].getSpeedRPM()); }
  Serial.printf("\nPulse:\t");
  for(int i=0;i<cnt;i++) { Serial.printf("\t%d", Servo[i].getPulse()); }
  // Delay 3 seconds
  delay(3000);
}
