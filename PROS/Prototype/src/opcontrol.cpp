#include "main.h"

void operatorControl() {
	while (1) {
		while (true) {
			drive::operatorControl();
			// arms::operatorControl();
			// claws::operatorControl();
			// mobileGoalIntake::operatorControl();
			delay(DRIVERCONTROL_LOOP_DELAY);
		}
	}
}
