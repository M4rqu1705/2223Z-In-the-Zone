#include "main.h"

void autonomous() {
	resetValues();

	while (drive::notDone) {
		if(drive::notDone == true) drive::move(drive::turnRight, 90, 75, false);
		delay(AUTON_LOOPS_DELAYS);
	}
	resetValues();
	delay(200);
	print("Turned right");

	while (drive::notDone) {
		if(drive::notDone == true) drive::move(drive::turnLeft, 180, 75, false);
		delay(AUTON_LOOPS_DELAYS);
	}
	resetValues();
	delay(200);
	print("Turned left");

	while (drive::notDone) {
		if(drive::notDone == true) drive::move(drive::turnRight, 90, 75, false);
		delay(AUTON_LOOPS_DELAYS);
	}
	resetValues();
	delay(200);
	print("Turned right");

	while (drive::notDone) {
		if(drive::notDone == true) drive::move(drive::backward, 5, 75, false);
		delay(AUTON_LOOPS_DELAYS);
	}
	resetValues();
	delay(200);
	print("Moved backward");

	while (drive::notDone) {
		if(drive::notDone == true) drive::move(drive::forward, 10, 75, false);
		delay(AUTON_LOOPS_DELAYS);
	}
	resetValues();
	delay(200);
	print("Moved forward");

	while (drive::notDone){
		if(drive::notDone == true) drive::move(drive::backward, 5, 75, false);
		delay(AUTON_LOOPS_DELAYS);
	}
	resetValues();
	print("Done!!");
}
