#include "WallClimbingRobot.h"

void setup() {  
  WallClimbingRobot::setup();
}

void loop() {
  delay(3000);
  WallClimbingRobot::turn(1);
  //WallClimbingRobot::traverseWall();
  //WallClimbingRobot::findWall();
  //WallClimbingRobot::findObject();
  //WallClimbingRobot::moveForward(1);
}
