#include "main.h"

void mobileGoalOperatorControl() {
	if (joystickGetDigital(1, JOYSTICK_MOGO, JOYSTICK_MOGO_BUTTON) == 1) {
		if (!mogoButtonPressed) {
			mogoButtonPressed = true;
			mogoCounter = (mogoCounter > MOGO_CYCLES || mogoCounter <= 0) ? 1 : mogoCounter;
			mogoRetracted = !mogoRetracted;
		}
	}
	else mogoButtonPressed = false;

	mobileGoalControl(mogoRetracted);
}


void mobileGoalControl(bool state) {

	mogoLoutput = state ? -127 : 127;
	mogoRoutput = state ? 127 : -127;

	if (mogoCounter <= MOGO_CYCLES) {
		motorSet(MOTOR_MOGO_L, mogoLoutput);
		motorSet(MOTOR_MOGO_R, mogoRoutput);
		mogoDone = false;
	}
	else if (mogoCounter > MOGO_CYCLES || mogoCounter <= 0) {
		motorSet(MOTOR_MOGO_L, 0);
		motorSet(MOTOR_MOGO_R, 0);
		mogoDone = true;
	}
	mogoCounter += (mogoCounter <= MOGO_CYCLES + 1) ? 1 : 0;
}
