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
	float prev_state = 0;

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
		//delay(50);
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

			motor1.setSpeed((int)(-speedFactor * MAX_MOTOR_SPEED));
			motor2.setSpeed((int)(-speedFactor * MAX_MOTOR_SPEED));
			motor3.setSpeed((int)(-speedFactor * MAX_MOTOR_SPEED));
			motor4.setSpeed((int)(-speedFactor * MAX_MOTOR_SPEED));
		}
	}

	void stop()
	{
		motor1.run(RELEASE);
		motor2.run(RELEASE);
		motor3.run(RELEASE);
		motor4.run(RELEASE);
	}

	// Note: 1.0668m to peak of wall
	void traverseWall()
	{
		Serial.println("Driving given distance...");
		driveDistance(0.9, 1);

		Serial.println("Driving full speed...");
		driveSpeed(1);
		
		Serial.println("Waiting for tilt switch change...");
		waitForTiltSwitchChange();

		Serial.println("Stopping...");
		stop();
		delay(500);

		//Serial.println("Driving over bump...");
		//driveDistance(0.1, 1);

		Serial.println("Slowly driving down first bit of wall...");
		driveDistance(0.3, 0.4);

		Serial.println("Quickly driving down rest of wall...");
		driveSpeed(1);

		Serial.println("Waiting for limit switch press...");
		waitForLimitSwitchPress();

		/*Serial.println("Driving forward to detach from wall...");
		driveDistance(0.25, 1);

		Serial.println("Stopping...");*/
		stop();

		Serial.println("Wall traversal complete! Gonna delay now");

		delay(5000);
	}

	void waitForTiltSwitchChange()
	{
		int prevStableState = getTiltSwitchState();
		//Serial.print("Previous stable tilt switch state: ");
		//Serial.println(prevStableState);

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
		/*int curState;
		int prevState = LOW;

		int lastStateChangeTime = millis();

		while (1)
		{
			curState = digitalRead(TILT_SWITCH_PIN);
			//Serial.print("Current tilt switch state: ");
			//Serial.println(curState);

			if (curState != prevState)
			{
				lastStateChangeTime = millis();
				prevState = curState;
			}

			// If tilt switch has settled
			if ((millis() - lastStateChangeTime) > TILT_SWITCH_DEBOUNCE_TIME)
			{
				//Serial.println("Tilt switch state settled!");
				break;
			}
		}

		return curState;*/
		float raw_state = digitalRead(TILT_SWITCH_PIN);
		int filtered_tilt;

		prev_state = prev_state * LOW_PASS_VALUE + raw_state * (1-LOW_PASS_VALUE);
		//	prev_state = new_state

		if(prev_state > TILT_SWITCH_THRESHOLD)
		{
			filtered_tilt=1;
		}
		else
		{
			filtered_tilt = 0;
		}

		Serial.print(raw_state);
		Serial.print(",");
		Serial.print(prev_state);
		Serial.print(",");
		Serial.println(filtered_tilt);

		return filtered_tilt;
	}

	void findWall()
	{ 
		turnDistance(-0.25); //left
		driveSpeed(0.8);
		waitForLimitSwitchPress();
		stop(); //boundary has been hit

		driveDistance(-0.55, 1);
		stop();
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
		Serial.println("Delay complete, start finding object");
		int prevTime = 0;
		int distToWall = 0;
		waitForLimitSwitchPress();
		Serial.println("Limit Switch Pressed");
		driveDistance(0.3, 1); //drive farther forward to straighten the tail
		stop();
		turnDistance(-0.25);
		driveSpeed(0.8);
		waitForLimitSwitchPress();
		stop();
		driveDistance(-0.07, 1); //back up just a little bit so the boundary or other weird things don't get detected
		stop();
		servo.write(5); //0 swings it too far
		delay(500);

		prevTime = millis();
		while((millis() - prevTime) < 2500)  //stablize value
		{
			readDistance();
			distToWall = (distToWall + distance)/2;
			Serial.print("distToWall: ");
			Serial.println(distToWall);
		}

		driveSpeed(-0.4); //drive slowly so the target is not missed

		int lastDistance = 0;
		int objectFound = 0; 
		int distanceToObject = 0;
		while(objectFound == 0) {
			Serial.print("Last Distance: ");
			Serial.println(lastDistance);
			lastDistance = (lastDistance + distance)/2;

			readDistance();
			if(lastDistance <= (distToWall - 25.0)){
				Serial.println(lastDistance);
				if(distance <= (lastDistance + 2) && distance >= (lastDistance - 2) && distance >= 0) {
					Serial.println(distance);
					objectFound = 1;
					distanceToObject = distance;
				}
			}
		}

		stop();
		driveDistance(-0.02, 1); //Object is usually detected early, so go a little more
		stop();
		turnDistance(0.25);
		Serial.println("Servo Turn");
		servo.write(90);
		delay(500);
		Serial.println("Going towards object");
		Serial.println(distanceToObject);
		driveDistance((distanceToObject + 10)/100.0, 1);	
		stop();
	}

	void returnToWall() {
		driveDistance(-0.12, 1);
		stop();
		turnDistance(-0.25);
		stop();
		driveSpeed(1);
		waitForLimitSwitchPress();
		driveDistance(-0.55, 1);
		stop();
		turnDistance(-0.25);
		driveSpeed(1);
		waitForLimitSwitchPress();
		stop();
	}

	void test() { 
		
		
	}

	void returnToBase1() { 
		int distToWall = 0; 
		int prevTime = 0;
		
		waitForLimitSwitchPress();
		driveDistance(0.2, 1);
		stop();
		turnDistance(0.25);
		driveSpeed(0.8);
		waitForLimitSwitchPress();
		driveDistance(-0.2, 1);
		turnDistance(-0.25);
		stop();
		servo.write(180);
		delay(500);

		prevTime = millis();
		while((millis() - prevTime) < 2500) //to stabilize value 
		{
			readDistance();
			distToWall = (distToWall + distance)/2;
			Serial.print("distToWall: ");
			Serial.println(distToWall);
		}

		driveSpeed(0.5);

		int lastDistance = 0;
		int objectFound = 0; 
		int distanceToObject = 0;
		while(objectFound == 0) {

			Serial.print("Last Distance: ");
			Serial.println(lastDistance);
			lastDistance = (lastDistance + distance)/2;

			readDistance();

			if(lastDistance <= (distToWall - 25.0)){
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
		driveDistance(0.02, 1); //Object is usually detected early, so go a little more
		stop();
		turnDistance(-0.25);
		Serial.println("Servo Turn");
		servo.write(90);
		delay(500);
		Serial.println("Going towards object");
		Serial.println(distanceToObject);
		driveDistance((distanceToObject + 15)/100.0, 1);
		stop();
	}

	void driveDistance(double distanceFactor, double speedFactor)
	{
		encCount = 0;
		int totalEncCount = ENC_DIR_FACTOR * (int)(distanceFactor * ENC_COUNT_PER_METRE);
		Serial.print("Encoder count to complete forward movement: ");
		Serial.println(totalEncCount);

		if (distanceFactor > 0) {
			driveSpeed(speedFactor);

			while(encCount > totalEncCount)
			{
				Serial.print("Encoder count: ");
				Serial.println(encCount);
			}
		} else { 
			driveSpeed(-speedFactor);

			while(encCount < totalEncCount)
			{
				Serial.print("Encoder count: ");
				Serial.println(encCount);
			
			}
		}

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
