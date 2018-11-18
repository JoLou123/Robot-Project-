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

	void driveGivenSpeed(double speedFactor)
	{
		motor1.run(FORWARD);
		motor4.run(FORWARD);
		motor2.run(FORWARD);
		motor3.run(FORWARD);

		motor1.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
		motor2.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
		motor3.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
		motor4.setSpeed((int)(speedFactor * MAX_MOTOR_SPEED));
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
		driveGivenDistance(0.5);

		Serial.println("Driving full speed...");
		driveGivenSpeed(1);
		
		Serial.println("Waiting for tilt switch change...");
		waitForTiltSwitchChange();

		Serial.println("Driving slower speed...");
		driveGivenSpeed(0.2);

		Serial.println("Waiting for limit switch press...");
		waitForLimitSwitchPress();

		Serial.println("Stopping...");
		stop();

		Serial.println("Wall traversal complete!");
	}

	void waitForTiltSwitchChange()
	{
		int lastTiltStateChangeTime = millis();
		int elapsedContinuousTimeAtNewState = 0;
		int debounceTime = 500; // Milliseconds

		int oldTiltState = digitalRead(TILT_SWITCH_PIN);

		int curTiltState;
		int prevTiltState = LOW;

		Serial.print("Previous tilt switch state: ");
		Serial.println(curTiltState);

		while (1)
		{
			curTiltState = digitalRead(TILT_SWITCH_PIN);
			Serial.print("Current tilt switch state: ");
			Serial.println(curTiltState);

			if (curTiltState != prevTiltState)
			{
				lastTiltStateChangeTime = millis();
				prevTiltState = curTiltState;
			}

			if (curTiltState != oldTiltState)
			{
				elapsedContinuousTimeAtNewState = millis() - lastTiltStateChangeTime;
			}
			else
			{
				elapsedContinuousTimeAtNewState = 0;
			}

			Serial.print("Elapsed continuous time at new state: ");
			Serial.println(elapsedContinuousTimeAtNewState);

			// If tilt switch has changed and settled
			if (elapsedContinuousTimeAtNewState > debounceTime)
			{
				Serial.println("Tilt switch state changed and settled!");
				return;
			}
		}

	void findWall()
	{ 
		turn(-0.25); //left
		goForward(255);
		checkLimit();
		stop(); //boundary has been hit

		drive(-0.55);
		turn(0.25);
		goForward(255);
		checkLimit();
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
		int prevTime;
		int distToWall = 0;

		Serial.println("Before Forward Drive");
		drive(0.1);
		Serial.println("After Forward Drive");
		turn(-0.25);

		goForward(255);
		checkLimit();	
		stop();

		Serial.println("Before Backward Drive");
		drive(-0.1);
		Serial.println("After Backward Drive");
		turn(0.25);

		goForward(255);
		checkLimit();	
		stop();

		drive(-0.1);
		turn(0.25);

		Serial.println("Turn Servo");
		servo.write(0);
		delay(500);

		stop();
		prevTime = millis();
		while((millis() - prevTime) < 3000) 
		{
			readDistance();
			distToWall = (distToWall + distance)/2;
			Serial.print("distToWall: ");
			Serial.println(distToWall);
		}

		goForward(150);

		while(distance >= (distToWall - 5.0)) {
			Serial.print("Distance: ");
			Serial.println(distance);
			readDistance();
		}

		stop();
		turn(0.25);
		Serial.println("Servo Turn");
		servo.write(90);
		delay(500);
		Serial.println("Going towards object");
		goForward(255);
		checkLimit();	
		stop();
	}

	void driveGivenDistance(double distanceFactor)
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
