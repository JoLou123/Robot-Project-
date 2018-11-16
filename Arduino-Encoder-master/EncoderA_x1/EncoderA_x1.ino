#include <AFMotor.h>

#define PULSES_PER_ROTATION (1450)
#define ENC_A_PIN_NUM (18)
#define ENC_B_PIN_NUM (19)

#define ROBOT_CENTRE_TO_WHEEL_DISTANCE (11) //cm
#define WHEEL_RADIUS (3.5) //cm
#define RADIUS_RATIO (ROBOT_CENTRE_TO_WHEEL_DISTANCE / WHEEL_RADIUS)

int encoderCount = 0;
int turnFinished = 0;

AF_DCMotor motor1(1, MOTOR12_64KHZ); // Create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ); // Create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR34_1KHZ); // Create motor #3, 1KHz pwm
AF_DCMotor motor4(4, MOTOR34_1KHZ); // Create motor #4, 1KHz pwm

void setup()
{
	Serial.begin(9600);

	pinMode(51, OUTPUT);
	digitalWrite(51, HIGH);

	pinMode(ENC_A_PIN_NUM, INPUT);
	pinMode(ENC_B_PIN_NUM, INPUT); 
	attachInterrupt(digitalPinToInterrupt(ENC_A_PIN_NUM), ENC_PHASE_A_CHANGE_ISR, CHANGE);
}

// Turn robot a fraction of a 360 degree turn
void turnRobot(int turnFraction)
{
	int numPulses = turnFraction * PULSES_PER_ROTATION * RADIUS_RATIO;

	// Right turn
	if(turnFraction > 0)
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

		while(encoderCount < numPulses) {
			Serial.print("Encoder count: ");
			Serial.println(encoderCount);
		}
	}

	// Left turn
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

		while(encoderCount > numPulses) {
			Serial.print("Encoder count: ");
			Serial.println(encoderCount);
		}
	}

	motor1.run(RELEASE);
	motor2.run(RELEASE);
	motor3.run(RELEASE);
	motor4.run(RELEASE);

	encoderCount = 0;
	turnFinished = 1;
}

void loop()
{
	/*
	if(encoderCount >= total)
	{
		motor3.setSpeed(0);
	}
	else
	{
		motor3.run(FORWARD);
		motor3.setSpeed(255);
		Serial.println(encoderCount);
	}
	*/

	if(turnFinished == 0)
	{
		turnRobot(1);
	}
}

// Service a change in encoder phase A logic level
void ENC_PHASE_A_CHANGE_ISR()
{
	if(digitalRead(ENC_B_PIN_NUM) == 0)
	{
		if (digitalRead(ENC_A_PIN_NUM) == 0)
		{
			// A fell, B is low
			encoderCount--; // Moving backwards
		}
		else
		{
			// A rose, B is low
			encoderCount++; // Moving forwards
		}
	}
}
