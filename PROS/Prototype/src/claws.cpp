#include <main.hpp>

namespace claws {
	void changePositionTo(bool state) {
		//Reevaluate the claw's output per side based on the current state of 'state'
		if (state) {
			outputs[0] = -CLAW_SPEED;
			outputs[1] = CLAW_SPEED;
		}
		else {
			outputs[0] = CLAW_SPEED;
			outputs[1] = -CLAW_SPEED;
		}

		//If CLAWS_CYCLES time has not gone by, keep moving claws
		if (counter < CLAWS_CYCLES) {
			motorSet(MOTOR_CLAW_L, outputs[0]);
			motorSet(MOTOR_CLAW_R, outputs[1]);
			counter += 1;
			notDone = true;
		}
		//If CLAWS_CYLES time has not gone by, stop moving claws and make clawsDone true
		else {
			motorSet(MOTOR_CLAW_L, 0);
			motorSet(MOTOR_CLAW_R, 0);
			notDone = false;
		}
	}

	void operatorControl() {
		//Toggle button to change the claw's position
		if (joystickGetDigital(1, JOYSTICK_CLAWS, JOYSTICK_CLAWS_BUTTON) == 1) {
			if (buttonPressed) {
				buttonPressed = true;
				openRight = !openRight;
				if (counter == CLAWS_CYCLES) counter = 0;
			}
		}
		else buttonPressed = false;

		//Call clawsControl function with updated state of claws
		changePositionTo(openRight);
	}

}
