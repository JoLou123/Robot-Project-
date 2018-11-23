#include "WallClimbingRobot.h"

void setup() {
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  
  WallClimbingRobot::setup();
}

void loop() {
  
  WallClimbingRobot::ledOn();
  WallClimbingRobot::waitForLimitSwitchPress();
  WallClimbingRobot::ledOff();

  WallClimbingRobot::findWall();
  WallClimbingRobot::traverseWall();
  
  delay(5000); // PULL ROBOT OFF WALL
  WallClimbingRobot::ledOn();
  WallClimbingRobot::waitForLimitSwitchPress(); // RESUME ROBOT
  WallClimbingRobot::ledOff();

  WallClimbingRobot::findObject();
  WallClimbingRobot::returnToWall();
  WallClimbingRobot::traverseWall();
  
  delay(5000); // PULL ROBOT OFF WALL
  WallClimbingRobot::ledOn();
  WallClimbingRobot::waitForLimitSwitchPress(); // RESUME ROBOT
  WallClimbingRobot::ledOff();

  WallClimbingRobot::returnToBase1();
  
  WallClimbingRobot::waitForLimitSwitchPress();
  while(1) {
  WallClimbingRobot::turnDistance(0.25);
  delay(1000);
  }
}
