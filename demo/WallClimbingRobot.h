#ifndef WALLCLIMBINGROBOT_H
#define WALLCLIMBINGROBOT_H

#define ENC_DIR_FACTOR (-1) // 1 for positive forwards, -1 for positive backwards

#define ENC_COUNT_PER_METRE (6635)
#define ENC_COUNT_PER_ROBOT_ROTATION (4062)

#define ENC_A_PIN_NUM (18)
#define ENC_B_PIN_NUM (19)

#define LEFT_LIMIT (25)

#define SERVO_PIN (44)

#define TRIG_PIN (42)
#define ECHO_PIN (40)

namespace WallClimbingRobot
{
	void setup();

	void findObject();

	void goForward(int speed);

	void stop();

	void checkLimit();

	void traverseWall();

	void findWall();

	// Move robot straight a factor of 1 metre
	void drive(double distanceFactor);

	// Turn robot a factor of a 360 degree turn
	// Positive for right turn, negative for left turn
	void turn(double turnFactor);

	// ISR for change in encoder A phase
	void ENC_PHASE_A_CHANGE_ISR();
};

#endif /* WALLCLIMBINGROBOT_H */
