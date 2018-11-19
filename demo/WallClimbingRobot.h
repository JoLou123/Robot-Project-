#ifndef WALLCLIMBINGROBOT_H
#define WALLCLIMBINGROBOT_H

#define MAX_MOTOR_SPEED (255)
#define ENC_DIR_FACTOR (-1) // 1 for positive forwards, -1 for positive backwards
#define ENC_COUNT_PER_METRE (6635)
#define ENC_COUNT_PER_ROBOT_ROTATION (4650)
#define TILT_SWITCH_DEBOUNCE_TIME (500) // milliseconds

#define ENC_A_PIN (18)
#define ENC_B_PIN (19)
#define LEFT_LIMIT_PIN (25)
#define SERVO_PIN (44)
#define TRIG_PIN (42)
#define ECHO_PIN (40)
#define TILT_SWITCH_PIN (26)

namespace WallClimbingRobot
{
	void setup();
	void findObject();

	// Move robot straight forward with a given factor of max speed
	void driveSpeed(double speedFactor);

	void stop();
	void traverseWall();

	// Move robot straight forward a factor of 1 metre
	void driveDistance(double distanceFactor);

	void findWall();

	// Turn robot a factor of a 360 degree turn
	// Positive for right turn, negative for left turn
	void turnGivenDistance(double turnFactor);

	void waitForLimitSwitchPress();
	void waitForTiltSwitchChange();

	// Get debounced tilt switch state
	int getTiltSwitchState();

	// ISR for change in encoder A phase
	void ENC_PHASE_A_CHANGE_ISR();
};

#endif /* WALLCLIMBINGROBOT_H */
