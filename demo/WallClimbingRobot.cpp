#include <Arduino.h>
#include <AFMotor.h>
#include "WallClimbingRobot.h"

namespace WallClimbingRobot
{
	AF_DCMotor motor1(1, MOTOR12_64KHZ); // Create motor #1, 64KHz pwm
	AF_DCMotor motor2(2, MOTOR12_64KHZ); // Create motor #2, 64KHz pwm
	AF_DCMotor motor3(3, MOTOR34_1KHZ); // Create motor #3, 1KHz pwm
	AF_DCMotor motor4(4, MOTOR34_1KHZ); // Create motor #4, 1KHz pwm

	int encCount;

	void setup()
	{
		Serial.begin(9600); // For debugging

		pinMode(ENC_A_PIN_NUM, INPUT);
		pinMode(ENC_B_PIN_NUM, INPUT);

		attachInterrupt(digitalPinToInterrupt(ENC_A_PIN_NUM), ENC_PHASE_A_CHANGE_ISR, CHANGE);
		encCount = 0;
		Serial.println("WallClimbingRobot setup complete");
	}

	void turn(double turnFactor)
	{
		int turnEncCount = -1 * (int)(turnFactor * ENC_COUNT_PER_ROBOT_ROTATION);
		Serial.print("Encoder count to complete turn: ");
		Serial.println(turnEncCount);

		// Right turn, encoder count decreases
		if (turnFactor > 0)
		{
			// Left motors
			motor1.run(FORWARD);
			motor1.setSpeed(255);
			motor4.run(FORWARD);
			motor4.setSpeed(255);

			// Right motors
			motor2.run(BACKWARD);
			motor2.setSpeed(255);
			motor3.run(BACKWARD);
			motor3.setSpeed(255);

			while(encCount > turnEncCount)
			{
				Serial.print("Encoder count: ");
				Serial.println(encCount);
			}
		}

		// Left turn, encoder count increases
		else
		{
			// Left motors
			motor1.run(BACKWARD);
			motor1.setSpeed(255);
			motor4.run(BACKWARD);
			motor4.setSpeed(255);

			// Right motors
			motor2.run(FORWARD);
			motor2.setSpeed(255);
			motor3.run(FORWARD);
			motor3.setSpeed(255);

			while(encCount < turnEncCount)
			{
				Serial.print("Encoder count: ");
				Serial.println(encCount);
			}
		}

		motor1.run(RELEASE);
		motor2.run(RELEASE);
		motor3.run(RELEASE);
		motor4.run(RELEASE);

		encCount = 0;
	}

	void move(double distanceFactor)
	{
		Serial.println("TODO: Implement move()");
	}

	void ENC_PHASE_A_CHANGE_ISR()
	{
		if(digitalRead(ENC_B_PIN_NUM) == 0)
		{
			if (digitalRead(ENC_A_PIN_NUM) == 0)
			{
				// A fell, B is low
				encCount--; // Moving backwards
			}
			else
			{
				// A rose, B is low
				encCount++; // Moving forwards
			}
		}
	}
}
