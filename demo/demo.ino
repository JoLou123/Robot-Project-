#include "WallClimbingRobot.h"

void setup() {
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
  WallClimbingRobot::setup();
}

void loop() {
  WallClimbingRobot::waitForLimitSwitchPress();
  WallClimbingRobot::findWall();
  WallClimbingRobot::traverseWall();
  WallClimbingRobot::findObject();
  WallClimbingRobot::returnToWall();
  WallClimbingRobot::traverseWall();
  WallClimbingRobot::returnToBase1();
   
  //delay(5000);
}
