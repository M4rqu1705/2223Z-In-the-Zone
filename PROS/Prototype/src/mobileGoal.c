#include "main.h"

void mobileGoalOperatorControl() {
	if (joystickGetDigital(1, JOYSTICK_MOGO, JOYSTICK_MOGO_BUTTON) == 1) {
		if (!mogoButtonPressed) {
			mogoButtonPressed = true;
			mogoRetracted = !mogoRetracted;
			if (mogoCounter > MOGO_CYCLES) mogoCounter = 1;
		}
	}
	else mogoButtonPressed = false;

	mobileGoalControl(mogoRetracted);
}


void mobileGoalControl(bool state) {
	if (state) {
		mogoLoutput = -127;
		mogoRoutput = 127;
	}
	else {
		mogoLoutput = 127;
		mogoRoutput = -127;
	}

	if (mogoCounter <= MOGO_CYCLES) {
		motorSet(MOTOR_MOGO_L, mogoLoutput);
		motorSet(MOTOR_MOGO_R, mogoRoutput);
		mogoDone = false;
	}
	else if (mogoCounter > MOGO_CYCLES) {
		motorSet(MOTOR_MOGO_L, 0);
		motorSet(MOTOR_MOGO_R, 0);
		mogoDone = true;
	}
	mogoCounter += (mogoCounter <= MOGO_CYCLES + 1) ? 1 : 0;
}
