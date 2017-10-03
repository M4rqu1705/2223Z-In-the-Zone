#include "main.h"

void operatorControl() {
	while (1) {

		//Drive
		driveOperatorControl();

		//Arm
		armsOperatorControl();

		//Claw
		clawsOperatorControl();

		//Mobile Goal
		mobileGoalOperatorControl();

		delay(DRIVERCONTROL_LOOP_DELAY);
	}
}
