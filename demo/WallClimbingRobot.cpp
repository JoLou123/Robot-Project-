#include <Arduino.h>
#include <AFMotor.h>
#include <Servo.h>
#include "WallClimbingRobot.h"

namespace WallClimbingRobot
{
	AF_DCMotor motor1(1, MOTOR12_64KHZ); // Create motor #1, 64KHz pwm
	AF_DCMotor motor2(2, MOTOR12_64KHZ); // Create motor #2, 64KHz pwm
	AF_DCMotor motor3(3, MOTOR34_1KHZ); // Create motor #3, 1KHz pwm
	AF_DCMotor motor4(4, MOTOR34_1KHZ); // Create motor #4, 1KHz pwm

	Servo servo;
	int pos;
	int distance;
	int duration;

	// Necessary global since it's updated by ISR
	int encCount;

	void setup()
	{
		Serial.begin(9600); // For debugging

		pinMode(ENC_A_PIN, INPUT);
		pinMode(ENC_B_PIN, INPUT);

		pinMode(LEFT_LIMIT_PIN, INPUT);
		pinMode(TRIG_PIN, OUTPUT);
		pinMode(ECHO_PIN, INPUT);

		servo.attach(SERVO_PIN);

		pinMode(TILT_SWITCH_PIN, INPUT);
  		digitalWrite(TILT_SWITCH_PIN, HIGH); // Internal pull-up
	
		attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), ENC_PHASE_A_CHANGE_ISR, CHANGE);
		encCount = 0;
		Serial.println("WallClimbingRobot setup complete");
	}

	void readDistance() {
		// Clears the trigPin
		digitalWrite(TRIG_PIN, LOW);
		delayMicroseconds(2);

		// Sets the trigPin to LOW,HIGH, then LOW state for clean signal
		digitalWrite(TRIG_PIN, LOW);
		delayMicroseconds(5);
		digitalWrite(TRIG_PIN, HIGH);
		delayMicroseconds(10);
		digitalWrite(TRIG_PIN, LOW);

		// Reads the echoPin, returns the sound wave travel time in microseconds
		duration = pulseIn(ECHO_PIN, HIGH);

		// Calculating the distance
		distance = duration*0.034/2;
	}

	void driveSpeed(double speedFactor)
	{
		if(speedFactor > 0) {
			motor1.run(FORWARD);
			motor4.run(FORWARD);
			motor2.run(FORWARD);
			motor3.run(FORWARD);

			motor1.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
			motor2.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
			motor3.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
			motor4.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
		} else { 
			motor1.run(BACKWARD);
			motor4.run(BACKWARD);
			motor2.run(BACKWARD);
			motor3.run(BACKWARD);

			motor1.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
			motor2.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
			motor3.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
			motor4.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
		}

	}

	void stop()
	{
		motor1.run(RELEASE);
		motor2.run(RELEASE);
		motor3.run(RELEASE);
		motor4.run(RELEASE);
	}

	// Note: 106.68cm to peak of wall
	void traverseWall()
	{
		Serial.println("Driving given distance...");
		driveDistance(0.5);

		Serial.println("Driving full speed...");
		driveSpeed(1);
		
		Serial.println("Waiting for tilt switch change...");
		waitForTiltSwitchChange();

		Serial.println("Driving slower speed...");
		driveSpeed(0.2);

		Serial.println("Waiting for limit switch press...");
		waitForLimitSwitchPress();

		driveDistance(0.1);

		Serial.println("Stopping...");
		stop();

		Serial.println("Wall traversal complete!");
	}

	void waitForTiltSwitchChange()
	{
		int prevStableState = getTiltSwitchState();
		Serial.print("Previous stable tilt switch state: ");
		Serial.println(prevStableState);

		int curStableState = prevStableState;

		while (curStableState == prevStableState)
		{
			curStableState = getTiltSwitchState();
			Serial.print("Current stable tilt switch state: ");
			Serial.println(curStableState);
		}

		Serial.println("Tilt switch state changed!");
	}

	int getTiltSwitchState()
	{
		int curState;
		int prevState = LOW;

		int lastStateChangeTime = millis();

		while (1)
		{
			curState = digitalRead(TILT_SWITCH_PIN);
			Serial.print("Current tilt switch state: ");
			Serial.println(curState);

			if (curState != prevState)
			{
				lastStateChangeTime = millis();
				prevState = curState;
			}

			// If tilt switch has settled
			if (millis() - lastStateChangeTime > TILT_SWITCH_DEBOUNCE_TIME)
			{
				Serial.println("Tilt switch state settled!");
				break;
			}
		}

		return curState;
	}

	void findWall()
	{ 
		turnDistance(-0.25); //left
		driveSpeed(0.8);
		waitForLimitSwitchPress();
		stop(); //boundary has been hit

		driveDistance(-0.55);
		turnDistance(0.25);
		driveSpeed(1);
		waitForLimitSwitchPress();
		stop();
	}

	void waitForLimitSwitchPress()
	{
		int left = 0;

		while(left == 0) 
		{
			left = digitalRead(LEFT_LIMIT_PIN);
			//Serial.println(left);
		}

		left = 0; 
	}

	void findObject() {
				int baseFound = 0; 
		int distToWall = 0; 
		int prevTime = 0;

		driveDistance(0.01);
		turnDistance(-0.25);
		driveSpeed(0.8);
		waitForLimitSwitchPress();
		stop();
		servo.write(5);
		delay(500);

		prevTime = millis();
		while((millis() - prevTime) < 2000) 
		{
			readDistance();
			distToWall = (distToWall + distance)/2;
			Serial.print("distToWall: ");
			Serial.println(distToWall);
		}

		driveSpeed(-0.5);

		int lastDistance = 0;
		int objectFound = 0; 
		int distanceToObject = 0;
		while(objectFound == 0) {
			Serial.print("Last Distance: ");
			Serial.println(lastDistance);
			lastDistance = (lastDistance + distance)/2;

			readDistance();
			if(lastDistance <= (distToWall-25.0)){
				Serial.println(lastDistance);
				if(distance <= (lastDistance + 2) && distance >= (lastDistance - 2)) {
					Serial.println(distance);
					objectFound = 1;
					distanceToObject = distance;
				}
			}
		}

		Serial.print("Distance Stopped: ");
		Serial.println(distance);
		stop();
		turnDistance(0.25);
		Serial.println("Servo Turn");
		servo.write(90);
		delay(500);
		Serial.println("Going towards object");
		Serial.println(distanceToObject);
		driveDistance((distanceToObject + 7)/100.0);	
		stop();
	}

	void returnToWall() {
		driveDistance(-0.12);
		turnDistance(0.25);
		driveSpeed(1);
		waitForLimitSwitchPress();
		driveDistance(-0.55);
		driveSpeed(1);
	}

	void test() { //Try finding base two from base 2 side of wall

		
	}

	void returnToBase1() { //Not Tested yet
		int prevTime = 0;
		int distToWall = 0; 

		driveSpeed(1);
		waitForLimitSwitchPress();
		driveDistance(-0.1);
		turnDistance(-0.25);
		servo.write(180);
		delay(500);

		prevTime = millis();
		while((millis() - prevTime) < 3000) 
		{
			readDistance();
			distToWall = (distToWall + distance)/2;
			Serial.print("distToWall: ");
			Serial.println(distToWall);
		}
		
		driveSpeed(0.5);

		while(distance >= (distToWall - 25.0)) {
			Serial.print("Distance: ");
			Serial.println(distance);
			readDistance();
		}

		turnDistance(-0.25);
		driveSpeed(1);
		waitForLimitSwitchPress();
	}

	void driveDistance(double distanceFactor)
	{
		encCount = 0;
		int totalEncCount = ENC_DIR_FACTOR * (int)(distanceFactor * ENC_COUNT_PER_METRE);
		Serial.print("Encoder count to complete forward movement: ");
		Serial.println(totalEncCount);

		if (distanceFactor > 0) {
			// Left motors
			motor1.run(FORWARD);
			motor1.setSpeed(MAX_MOTOR_SPEED);
			motor4.run(FORWARD);
			motor4.setSpeed(MAX_MOTOR_SPEED);

			// Right motors
			motor2.run(FORWARD);
			motor2.setSpeed(MAX_MOTOR_SPEED);
			motor3.run(FORWARD);
			motor3.setSpeed(MAX_MOTOR_SPEED);

			while(encCount > totalEncCount)
			{
				Serial.print("Encoder count: ");
				Serial.println(encCount);
			}
		} else { 
			// Left motors
			motor1.run(BACKWARD);
			motor1.setSpeed(MAX_MOTOR_SPEED);
			motor4.run(BACKWARD);
			motor4.setSpeed(MAX_MOTOR_SPEED);

			// Right motors
			motor2.run(BACKWARD);
			motor2.setSpeed(MAX_MOTOR_SPEED);
			motor3.run(BACKWARD);
			motor3.setSpeed(MAX_MOTOR_SPEED);

			while(encCount < totalEncCount)
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

	void turnDistance(double turnFactor)
	{
		encCount = 0;
		int totalEncCount = ENC_DIR_FACTOR * (int)(turnFactor * ENC_COUNT_PER_ROBOT_ROTATION);
		Serial.print("Encoder count to complete turn: ");
		Serial.println(totalEncCount);

		// Right turn, encoder count decreases
		if (turnFactor > 0)
		{
			// Left motors
			motor1.run(FORWARD);
			motor1.setSpeed(MAX_MOTOR_SPEED);
			motor4.run(FORWARD);
			motor4.setSpeed(MAX_MOTOR_SPEED);

			// Right motors
			motor2.run(BACKWARD);
			motor2.setSpeed(MAX_MOTOR_SPEED);
			motor3.run(BACKWARD);
			motor3.setSpeed(MAX_MOTOR_SPEED);

			while(encCount > totalEncCount)
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
			motor1.setSpeed(MAX_MOTOR_SPEED);
			motor4.run(BACKWARD);
			motor4.setSpeed(MAX_MOTOR_SPEED);

			// Right motors
			motor2.run(FORWARD);
			motor2.setSpeed(MAX_MOTOR_SPEED);
			motor3.run(FORWARD);
			motor3.setSpeed(MAX_MOTOR_SPEED);

			while(encCount < totalEncCount)
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

	void ENC_PHASE_A_CHANGE_ISR()
	{
		if(digitalRead(ENC_B_PIN) == 0)
		{
			if (digitalRead(ENC_A_PIN) == 0)
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
