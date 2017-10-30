#include <mogoIntake.hpp>

namespace mobileGoalIntake{
void changePositionTo(bool state) {
	if (state) {
		outputs[0] = -127;
		outputs[1] = 127;
	}
	else {
		outputs[0] = 127;
		outputs[1] = -127;
	}

	if (counter < MOGO_CYCLES) {
		motorSet(MOTOR_MOGO_L, outputs[0]);
		motorSet(MOTOR_MOGO_R, outputs[1]);
		counter += 1;
		notDone = false;
	}
	else if (counter == MOGO_CYCLES) {
		motorSet(MOTOR_MOGO_L, 0);
		motorSet(MOTOR_MOGO_R, 0);
		notDone = true;
	}
}

void operatorControl() {
	if (joystickGetDigital(1, JOYSTICK_MOGO, JOYSTICK_MOGO_BUTTON) == 1) {
		if (!buttonPressed) {
			buttonPressed = true;
			extend = !extend;
			if (counter == MOGO_CYCLES) counter = 0;
		}
	}
	else buttonPressed = false;

	changePositionTo(extend);
}

}
