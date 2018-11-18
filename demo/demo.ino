#include "WallClimbingRobot.h"

void setup() {  
  WallClimbingRobot::setup();
}

void loop() {
  delay(3000);
  
  WallClimbingRobot::traverseWall();
  //WallClimbingRobot::findObject();
  //WallClimbingRobot::moveForward(1);
}
