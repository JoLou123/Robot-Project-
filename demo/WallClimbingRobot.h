#ifndef WALLCLIMBINGROBOT_H
#define WALLCLIMBINGROBOT_H

#define ENC_COUNT_PER_ROBOT_ROTATION (6000)
#define ENC_A_PIN_NUM (18)
#define ENC_B_PIN_NUM (19)

namespace WallClimbingRobot
{
	void setup();

	// Turn robot a factor of a 360 degree turn
	// Positive for right turn, negative for left turn
	void turn(double turnFactor);

	// Move robot straight a factor of 1 metre
	void move(double distanceFactor);

	// ISR for change in encoder A phase
	void ENC_PHASE_A_CHANGE_ISR();
};

#endif /* WALLCLIMBINGROBOT_H */