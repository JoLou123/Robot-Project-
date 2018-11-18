#include "WallClimbingRobot.h"

void setup() {
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
  WallClimbingRobot::setup();
}

void loop() {
  //WallClimbingRobot::waitForTiltSwitchChange();
  //delay(4000);
  WallClimbingRobot::waitForLimitSwitchPress();
  WallClimbingRobot::traverseWall();
  delay(4000);
}
