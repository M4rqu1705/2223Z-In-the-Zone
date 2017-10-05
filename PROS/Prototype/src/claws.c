#include "main.h"

void clawsOperatorControl() {
	if (joystickGetDigital(1, JOYSTICK_CLAWS, JOYSTICK_CLAWS_BUTTON) == 1) {
		if (!clawButtonPressed) {
			clawButtonPressed = true;
			rightClawClosed = !rightClawClosed;
			if (clawsCounter > CLAWS_CYCLES) clawsCounter = 1;
		}
	}
	else clawButtonPressed = false;

	clawsControl(rightClawClosed);
}

void clawsControl(bool state) {
	if (state) {
		clawLoutput = -CLAW_SPEED;
		clawRoutput = CLAW_SPEED;
	}
	else {
		clawLoutput = CLAW_SPEED;
		clawRoutput = -CLAW_SPEED;
	}

	if (clawsCounter <= CLAWS_CYCLES) {
		motorSet(MOTOR_CLAW_L, clawLoutput);
		motorSet(MOTOR_CLAW_R, clawRoutput);
		clawsDone = false;
	}
	else if (clawsCounter > CLAWS_CYCLES) {
		motorSet(MOTOR_CLAW_L, 0);
		motorSet(MOTOR_CLAW_R, 0);
		clawsDone = true;
	}

	clawsCounter += (clawsCounter <= CLAWS_CYCLES + 1) ? 1 : 0;
}
