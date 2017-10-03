#include "main.h"

void clawsOperatorControl() {
	if (joystickGetDigital(1, JOYSTICK_CLAWS, JOYSTICK_CLAWS_BUTTON) == 1) {
		if (!clawButtonPressed) {
			clawButtonPressed = true;
			clawsCounter = (clawsCounter > CLAWS_CYCLES || clawsCounter <= 0) ? 1 : clawsCounter;
			rightClawClosed = !rightClawClosed;
		}
	}
	else clawButtonPressed = false;

	clawsControl(rightClawClosed);
}

void clawsControl(bool state) {

	clawLoutput = state ? -CLAW_SPEED : CLAW_SPEED;
	clawRoutput = state ? CLAW_SPEED : -CLAW_SPEED;

	if (clawsCounter <= CLAWS_CYCLES) {
		motorSet(MOTOR_CLAW_L, clawLoutput);
		motorSet(MOTOR_CLAW_R, clawRoutput);
		clawsDone = false;
	}
	else if (clawsCounter > CLAWS_CYCLES || clawsCounter <= 0) {
		motorSet(MOTOR_CLAW_L, 0);
		motorSet(MOTOR_CLAW_R, 0);
		clawsDone = true;
	}

	clawsCounter += (clawsCounter <= CLAWS_CYCLES + 1) ? 1 : 0;
}
