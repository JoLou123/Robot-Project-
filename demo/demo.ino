#include "WallClimbingRobot.h"

void setup() {
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
  WallClimbingRobot::setup();
}

void loop() {
  delay(3000);
  WallClimbingRobot::findObject();
}
