/*
 * OBJECT AVOIDANCE WITH SERVO
 * 
 * Robot will avoid objects in front.
 * Ultrasonic sensor scans for the best route to take
 * Robot can't detect objects at it's side, best to install 2 IR Detectors
 *
 * Best used on OSEPP Mechanical Kits.
 * TANK-01
 * TRITANK-01
 *
 * created 08 Nov 2016
 * by Sui Wei
 *
 */
 
#include "sensorsDRV.h"
#include <avr/wdt.h>
#include "TBMotor.h"
#include <Servo.h>

OseppTBMotor Motor1(12, 11);
OseppTBMotor Motor2(8, 3);
//OseppTBMotor Motor3(7, 6);
//OseppTBMotor Motor4(4, 5);

#define leftMotor Motor1
#define rightMotor Motor2

Ultrasonic ults(2, 4);

const int servoPin = 9;
const int servoBais = 85;

Servo sv;

int leftSpeed = 0;
int rightSpeed = 0;

void setup()
{
  //Setup a watchdog
  //When the battery voltage is insufficient / program unexpected
  //Watchdog reset chip
  wdt_enable(WDTO_4S);

  sv.attach(servoPin);
  sv.write(servoBais);
}

void loop()
{
  const float threshold = 300;

  //If in 4 seconds,The program is not coming back here.
  //Chip will reset
  wdt_reset();
  sv.write(servoBais);
  if (dist_filter() < threshold)
  {
    //If an obstacle is detected
    //stop the car first
    leftSpeed = rightSpeed = 0;
    SetMotor();
    if (moreOpenDirection() == 'R')
    {
      //If the right side is more open, turn right.
      leftSpeed = 255;
      rightSpeed = -255;
    } else {
      leftSpeed = -255;
      rightSpeed = 255;
    }
    SetMotor();
    //Keep turning until you avoid it
    while (dist_filter() < threshold) delay(50);
    //The obstacle is not in the range of ultrasonic sensor
    //It's probably still on the Tank route
    //Continue to turn out for some time, may be able to completely avoid it
    //Time delay depends on the speed of the tank.
    delay(300);
    return;//to the beginning of the loop function.
  }
  //If there is no problem, keep moving forward.
  leftSpeed = rightSpeed = 255;
  SetMotor();
}

//Multiple sampling, take the average, is a good way to resist interference
float dist_filter()
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
}

char moreOpenDirection()
{
  long scan[90];
  sv.write(servoBais);
  //Empty array
  for (int i = 0; i < 90; i++)scan[i] = 0;
  delay(100);
  //Scan the distance around and into the array
  for (int i = 0; i > -45; i--)
  {
    sv.write(servoBais + i); delay(2);
    scan[45 - i] = ults.Detect();
  }
  for (int i = -45; i < 45; i++)
  {
    sv.write(i + servoBais); delay(2);
    scan[45 - i - 1] += ults.Detect();
  }
  for (int i = 45; i > 0; i--)
  {
    sv.write(i + servoBais); delay(2);
    scan[45 - i] += ults.Detect();
  }
  //note:Every angle has been scanned for two times.
  sv.write(servoBais);

  //Calculate the side of the more open
  //We can simply sum the distance between two sides.
  long left = 0;
  for (int i = 0; i < 45; i++)left += scan[i];
  long right = 0;
  for (int i = 45; i < 90; i++)right += scan[i];
  if (left > right)return 'L'; else return 'R';
}



