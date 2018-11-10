#include <Arduino.h>
#include "TBMotor.h"

OseppTBMotor::OseppTBMotor(uint8_t dir_pin, uint8_t pwm_pin)
{
  _dir_pin=dir_pin;
  _pwm_pin=pwm_pin;
  pinMode(_dir_pin,OUTPUT);
  digitalWrite(_pwm_pin,LOW);
}
void OseppTBMotor::forward(uint8_t speed)
{
  digitalWrite(_dir_pin,HIGH);
  analogWrite( _pwm_pin,speed);
}
void OseppTBMotor::backward(uint8_t speed)
{
  digitalWrite(_dir_pin,LOW);
  analogWrite( _pwm_pin,speed);
}
void OseppTBMotor::speed(int _speed)
{
  if(_speed<0)
  {
    _speed=-_speed;
    digitalWrite(_dir_pin,LOW);
  }else{
    digitalWrite(_dir_pin,HIGH);
  }
  if(_speed>255)_speed=255;
  analogWrite( _pwm_pin,_speed);
}

