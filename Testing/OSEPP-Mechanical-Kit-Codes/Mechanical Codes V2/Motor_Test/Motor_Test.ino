#include "TBMotor.h"

OseppTBMotor Motor1(12, 11);
OseppTBMotor Motor2(8, 3);
//OseppTBMotor Motor3(7, 6);
//OseppTBMotor Motor4(4, 5);

#define leftMotor Motor1
#define rightMotor Motor2

void setup()
{
  
}

void loop()
{
  leftMotor.SetSpeed(255);
  rightMotor.SetSpeed(255);

}

//Multiple sampling, take the average, is a good way to resist interference
/*float dist_filter()
{
  float avgDist = 0;
  int i = 0;
  for (i = 0; i < 10; i++)avgDist += ults.Detect();
  return avgDist / i;
}

void SetMotor()
{
  if (leftSpeed > 255)leftSpeed = 255; else if (leftSpeed < -255)leftSpeed = -255;
  if (rightSpeed > 255)rightSpeed = 255; else if (rightSpeed < -255)rightSpeed = -255;
  //Depending on your connection,
  //if the direction of the motor rotation is not the direction you want,
  //you can change it by changing the Positive/negative sign of the speed
  leftMotor.SetSpeed(leftSpeed);
  rightMotor.SetSpeed(-rightSpeed);
}*/
