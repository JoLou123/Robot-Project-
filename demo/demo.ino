#include "WallClimbingRobot.h"

void setup() {
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
  WallClimbingRobot::setup();
}

void loop() {
  delay(5000);
  //WallClimbingRobot::findWall();
  //WallClimbingRobot::waitForLimitSwitchPress();
  //WallClimbingRobot::test();
  //WallClimbingRobot::traverseWall();
  WallClimbingRobot::findObject();
  //WallClimbingRobot::returnToWall();
  //WallClimbingRobot::traverseWall();
  //WallClimbingRobot::returnToBase1();
    
}
