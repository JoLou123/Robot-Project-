#include <Arduino.h>
#include "oseppRemote.h"

OseppRemote::OseppRemote()
{
  letter = 0;
  number = 0;
  minusSign = 0;
  mButtonState = 0;
  tempButton = 0;
  mLast_Data_millis = millis();
  mButtonHandle = NULL;
  mLeftJoystickHandle = NULL;
  mRightJoystickHandle = NULL;
  mGravityHandle = NULL;
}
void OseppRemote::attachButtonChange(button_change_handele handle) {
  mButtonHandle = handle;
}
void OseppRemote::attachLeftJoystickChange(joystick_change_handle handle) {
  mLeftJoystickHandle = handle;
}
void OseppRemote::attachRightJoystickChange(joystick_change_handle handle) {
  mRightJoystickHandle = handle;
}
void OseppRemote::attachGravityChange(gravity_change_handle handle) {
  mGravityHandle = handle;
}
void OseppRemote::feed(char data) {
  mLast_Data_millis = millis();
  if ((data >= '0') && (data <= '9'))
  {
    number *= 10;
    number += data - '0';
  } else if (data == '-') {
    minusSign = 1;
  } else {
    if (minusSign)number = -number;
    switch (letter)
    {
      case 'x':
        mChannel[CHANNEL_GRAVITY_X] = number;
        break;
      case 'y':
        mChannel[CHANNEL_GRAVITY_Y] = number;
        break;
      case 'z':
        mChannel[CHANNEL_GRAVITY_Z] = number;
        break;
      case 'u':
        mChannel[CHANNEL_LEFTJOYSTICK_X] = number;
        break;
      case 'v':
        mChannel[CHANNEL_LEFTJOYSTICK_Y] = number;
        break;
      case 'w':
        mChannel[CHANNEL_RIGHTJOYSTICK_X] = number;
        break;
      case 'a':
        mChannel[CHANNEL_RIGHTJOYSTICK_Y] = number;
        break;
      default:
        break;
    }
    number = 0;
    minusSign = 0;
    switch (data) {
      case 'L':
        tempButton |= BUTTON_L;
        break;
      case 'R':
        tempButton |= BUTTON_R;
        break;
      case 'U':
        tempButton |= BUTTON_U;
        break;
      case 'D':
        tempButton |= BUTTON_D;
        break;
      case 'X':
        tempButton |= BUTTON_X;
        break;
      case 'Y':
        tempButton |= BUTTON_Y;
        break;
      case 'A':
        tempButton |= BUTTON_A;
        break;
      case 'B':
        tempButton |= BUTTON_B;
        break;
      case '\n':
        if (mButtonHandle != NULL) {
          uint8_t changed = mButtonState ^ tempButton;
          mButtonHandle(changed & tempButton, changed & (~tempButton));
        }
        mButtonState = tempButton;
        tempButton = 0;

        if (mLeftJoystickHandle != NULL) {
          mLeftJoystickHandle(mChannel[CHANNEL_LEFTJOYSTICK_X], mChannel[CHANNEL_LEFTJOYSTICK_Y]);
        }
        if (mRightJoystickHandle != NULL) {
          mRightJoystickHandle(mChannel[CHANNEL_RIGHTJOYSTICK_X], mChannel[CHANNEL_RIGHTJOYSTICK_Y]);
        }
        if (mGravityHandle != NULL) {
          mGravityHandle(mChannel[CHANNEL_GRAVITY_X], mChannel[CHANNEL_GRAVITY_Y], mChannel[CHANNEL_GRAVITY_Z]);
        }
        break;
      default:
        break;
    }
    letter = data;
  }
}

uint8_t OseppRemote::getButtonState(uint8_t mask)
{
  return mButtonState & mask;
}
int OseppRemote::getChannalData(uint8_t channel)
{
  return mChannel[channel];
}

bool OseppRemote::isTimeoutFor(unsigned long howlong)
{
  if (millis() - mLast_Data_millis > howlong)return true;
  return false;
}
