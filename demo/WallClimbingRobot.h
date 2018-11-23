#ifndef WALLCLIMBINGROBOT_H
#define WALLCLIMBINGROBOT_H

#define MAX_MOTOR_SPEED (255)
#define ENC_DIR_FACTOR (-1) // 1 for positive forwards, -1 for positive backwards
#define ENC_COUNT_PER_METRE (6635)
#define ENC_COUNT_PER_ROBOT_ROTATION (4800) //4650 for sticky
#define TILT_SWITCH_DEBOUNCE_TIME (50) // milliseconds
#define LOW_PASS_VALUE (0.9)
#define TILT_SWITCH_THRESHOLD (0.7)
#define DIST_TO_WALL (219) //around this value 
#define DIST_FROM_WALL_TO_BOUNDARY (20.0)
#define DIST_TO_RAMP (110) // Actual value of 130, using conservative value since we just need to be under it
#define SCAN_INTERVAL_DIST (0.07)


#define ENC_A_PIN (18)
#define ENC_B_PIN (19)
#define LEFT_LIMIT_PIN (25)
#define SERVO_PIN (44)
#define TRIG_PIN (42)
#define ECHO_PIN (40)
#define TILT_SWITCH_PIN (50)
#define LED_PIN (52)

namespace WallClimbingRobot
{
	void setup();
	void findObject();

	// Move robot straight forward with a given factor of max speed
	void driveSpeed(double speedFactor);

	void stop();
	void traverseWall();
	void test();

	void returnToBase1Ultrasonic();
	void returnToBase1();

	// Move robot straight forward a factor of 1 metre
	// Doesn't stop after driving the distance
	// NOTE: Use only positive values for speedFactor
	// ALWAYS REMEMBER TO ADD A STOP AFTER THIS COMMAND
	void driveDistance(double distanceFactor, double speedFactor);

	void findWall();
	void returnToWall();

	// Turn robot a factor of a 360 degree turn
	// Positive for right turn, negative for left turn
	void turnDistance(double turnFactor);

	void waitForLimitSwitchPress();
	void waitForTiltSwitchChange();

	int readDistance();

	// Get average ultrasonic distance (cm) over a period of time
	int getAverageDistance(unsigned int samplingTime);

	// Get debounced tilt switch state
	int getTiltSwitchState();

	int scanForObject(int referenceDist, double driveDist);

	void ledOn();
	void ledOff();

	// ISR for change in encoder A phase
	void ENC_PHASE_A_CHANGE_ISR();
};

#endif /* WALLCLIMBINGROBOT_H */
