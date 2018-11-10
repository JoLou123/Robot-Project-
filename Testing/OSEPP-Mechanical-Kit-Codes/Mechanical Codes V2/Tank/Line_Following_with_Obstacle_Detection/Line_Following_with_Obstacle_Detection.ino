/*
 * SINGLE LINE FOLLOWER WITH ULTRASONIC SENSOR
 * 
 * Uses 5 line sensors.
 * Best used on a white surface ground with black line as tracks.
 * Follows a single track line.
 * Uses an ultrasonic sensor to STOP the Robot when there's an object in front.
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
const int servoBais = 110;
Servo sv;

int leftSpeed = 0;
int rightSpeed = 0;

void setup()
{
  for (int i = A0; i < A5; i++)pinMode(i, INPUT_PULLUP);
  sv.attach(servoPin);
  sv.write(servoBais);

  //Setup a watchdog
  //When the battery voltage is insufficient / program unexpected
  //Watchdog reset chip
  wdt_enable(WDTO_4S);

}

bool lineTracker[5];

void loop()
{
  const float threshold = 150;

  //If in 4 seconds,The program is not coming back here.
  //Chip will reset
  wdt_reset();
  sv.write(servoBais);
  for (int i = 0; i < 5; i++)lineTracker[i] = !digitalRead(i + A0); //delete "!" to follow a white line.


  if (ults.Detect() < threshold)
  {
    leftSpeed = 0;
    rightSpeed = 0;
    SetMotor();
    return;//to the beginning of the loop function.
  }
  int left = lineTracker[0] * 3 + lineTracker[1] * 2 + lineTracker[2]  - lineTracker[3] * 2 - lineTracker[4] * 3;
  int right = -lineTracker[0] * 3 - lineTracker[1] * 2 + lineTracker[2]  + lineTracker[3] * 2 + lineTracker[4] * 3;
  if (left > right)
  {
    leftSpeed = 255;
    rightSpeed = -255;
  } else if (left < right)
  {
    leftSpeed = -255;
    rightSpeed = 255;
  } else {
    leftSpeed = rightSpeed = 255;
  }
  SetMotor();
  //delay(25);
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





