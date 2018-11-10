/*
 * LIGHT FOLLOWER WITH OBJECTIVE AVOIDANCE
 * 
 * Robot goes toward an area with the most intensed light.
 * Robot will try and avoid when an object is in front
 * Robot works best with a room with different light intensity
 *
 * Best used on OSEPP Mechanical Kits.
 * TANK-01
 * TRITANK-01
 *
 * created 08 Nov 2016
 * by Sui Wei
 *
 */
//includes our sensor and motor codes
#include <avr/wdt.h>
#include "TBMotor.h"
#include <Servo.h>

OseppTBMotor Motor1(12, 11); //Left Motor
OseppTBMotor Motor2(8, 3);  //Right Motor
//OseppTBMotor Motor3(7, 6);
//OseppTBMotor Motor4(4, 5);

#define leftMotor Motor1
#define rightMotor Motor2

const int servoPin = 9;
const int servoBias = 90;
Servo sv;


int leftSpeed = 0;
int rightSpeed = 0;


float ping()
{
	digitalWrite(2, LOW);
	delayMicroseconds(2);
	digitalWrite(2, HIGH);
	delayMicroseconds(10);
	digitalWrite(2, LOW);
	unsigned long trigTime= pulseIn(4, HIGH, 2000);
	if (trigTime == 0)return 2000.0 / 5.8; else return (float)trigTime / 5.8;
}


void setup()
{
	//Setup a watchdog
	//When the battery voltage is insufficient / program unexpected
	//Watchdog reset chip
	wdt_enable(WDTO_4S);
	wdt_reset();
	for (int i = 0; i < 5; i++)pinMode(A0 + i, INPUT);
	pinMode(2, OUTPUT);
	pinMode(4, INPUT_PULLUP);
	sv.attach(servoPin);
	sv.write(servoBias);
}


void loop()
{
	//If program is not coming back here in time
	//Chip will reset
	wdt_reset();
	sv.write(servoBias);
	if (ping() < 220.0)
	{
		leftSpeed = rightSpeed = 0;
		SetMotor();
		if (moreOpenDirection() == 'R')
		{
			//If the right side is more open, turn right.
			leftSpeed = 255;
			rightSpeed = -255;
		}
		else {
			leftSpeed = -255;
			rightSpeed = 255;
		}
		SetMotor();
		//Keep turning until you avoid it
		while (ping() < 200.0) delay(50);
		//The obstacle is not in the range of ultrasonic sensor
		//It's probably still on the Tank route
		//Continue to turn out for some time, may be able to completely avoid it
		//Time delay depends on the speed of the tank.
		delay(300);
	}

	SetSpeedByFollower_Analog();
	deadCycleLog(leftSpeed, rightSpeed);
	if (carInDeadCycle())
	{
		leftSpeed = 200;
		rightSpeed = -leftSpeed;
		delay((rand() % 600) + 400);
	}
	else {
		SetMotor();
		delay(50);
	}
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




void SetSpeedByFollower_Analog()
{
	int follower[6];
	for (int i = 0; i < 6; i++)follower[i] = 0x3ff-analogRead(A0 + i);

	int left = (follower[3] - follower[5] - follower[4] + (follower[2] + follower[1] + follower[0]) / 3);
	int right = (follower[2] - follower[0] - follower[1] + (follower[3] + follower[4] + follower[5]) / 3);
	if ((left > 0) && (right > 0))
	{
		leftSpeed = 255;
		rightSpeed = 255;
	}
	else if ((left < 0) && (right < 0))
	{
		left = (follower[3] + follower[4] + follower[5]);
		right = (follower[0] + follower[1] + follower[2]);
		if (left > right)
		{
			leftSpeed = -255;
			rightSpeed = 255;
		}
		else
		{
			leftSpeed = 255;
			rightSpeed = -255;
		}
	}
	else 
	{
		if (left > 0) leftSpeed = 255; else leftSpeed = -255;
		if (right > 0)rightSpeed = 255; else rightSpeed = -255;
	}
}

char moreOpenDirection()
{
	long scan[90];
	sv.write(servoBias);
	//Empty array
	for (int i = 0; i < 90; i++)scan[i] = 0;
	delay(100);
	//Scan the distance around and into the array
	for (int i = 0; i > -45; i--)
	{
		sv.write(servoBias + i); delay(2);
		scan[45 - i] = ping();
	}
	for (int i = -45; i < 45; i++)
	{
		sv.write(i + servoBias); delay(2);
		scan[45 - i - 1] += ping();
	}
	for (int i = 45; i > 0; i--)
	{
		sv.write(i + servoBias); delay(2);
		scan[45 - i] += ping();
	}
	//note:Every angle has been scanned for two times.
	sv.write(servoBias);

	//Calculate the side of the more open
	//We can simply sum the distance between two sides.
	long left = 0;
	for (int i = 0; i < 45; i++)left += scan[i];
	long right = 0;
	for (int i = 45; i < 90; i++)right += scan[i];
	if (left > right)return 'L'; else return 'R';
}
