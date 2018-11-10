
#include <Arduino.h>
#include "oseppRemote.h"
#include "TBMotor.h"

OseppRemote remote;
OseppTBMotor Motor1(12, 11);
OseppTBMotor Motor2(8, 3);
//OseppTBMotor Motor3(7, 6);
//OseppTBMotor Motor4(4, 5);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void setMotors(int left, int right) {
  Motor1.speed(left);
  Motor2.speed(-right);
  Serial.print("L:["); Serial.print(left);
  Serial.print("]\t");
  Serial.print("R["); Serial.print(right);
  Serial.println("]");
}


const int controlMode_Button = 0;
const int controlMode_Joystick = 1;
const int controlMode_Gravity = 2;
int controlMode = controlMode_Button;


void loop() {
  // put your main code here, to run repeatedly:
  delay(40);//oseppRemote send data per 40ms
  while (Serial.available() > 0)remote.feed(Serial.read());
  if (remote.isTimeoutFor(200)) {
    setMotors(0, 0);
    return;
  }
  if (remote.getButtonState(BUTTON_Y)) {
    controlMode = controlMode_Button;
  } else if (remote.getButtonState(BUTTON_X)) {
    controlMode = controlMode_Joystick;
  } else if (remote.getButtonState(BUTTON_A)) {
    controlMode = controlMode_Gravity;
  }

  if (controlMode == controlMode_Button) {
    if (remote.getButtonState(BUTTON_U)) {
      setMotors(255, 255);
    } else if (remote.getButtonState(BUTTON_D)) {
      setMotors(-255, -255);
    } else if (remote.getButtonState(BUTTON_L)) {
      setMotors(-255, 255);
    } else if (remote.getButtonState(BUTTON_R)) {
      setMotors(255, -255);
    } else {
      setMotors(0, 0);
    }
  } else if (controlMode == controlMode_Joystick) {
    int x = remote.getChannalData(CHANNEL_LEFTJOYSTICK_X);
    int y = remote.getChannalData(CHANNEL_LEFTJOYSTICK_Y);
    int l, r;
    l = r = y;
    l += x;
    r -= x;
    int m = max(abs(l), abs(r));
    if (m > 255) {
      l = map(l, -m, m, -255, 255);
      r = map(r, -m, m, -255, 255);
    }
    setMotors(l, r);
  } else {
    int x = remote.getChannalData(CHANNEL_GRAVITY_X);
    int y = remote.getChannalData(CHANNEL_GRAVITY_Y);
    int z = remote.getChannalData(CHANNEL_GRAVITY_Z);
    y += 127;
    y = constrain(y, -127, 127);
    y = map(y, -127, 127, -255, 255);
    int l, r;
    l = r = y;
    l += x;
    r -= x;
    int m = max(abs(l), abs(r));
    if (m > 255) {
      l = map(l, -m, m, -255, 255);
      r = map(r, -m, m, -255, 255);
    }
    setMotors(l, r);
  }
}
