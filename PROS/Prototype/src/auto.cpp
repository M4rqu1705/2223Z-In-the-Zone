#include <main.hpp>

void autonomous() {
	while (drive::notDone) {
		drive::move(drive::turnRight, 90, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	print("Turned right");

	while (drive::notDone) {
		drive::move(drive::turnLeft, 180, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	print("Turned left");

	while (drive::notDone) {
		drive::move(drive::turnRight, 90, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	print("Turned right");

	while (drive::notDone) {
		drive::move(drive::backward, 5, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	print("Moved backward");

	while (drive::notDone) {
		drive::move(drive::forward, 10, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	print("Moved forward");

	while (drive::notDone){
	drive::move(drive::backward, 5, 75, false);
	delay(1);
	}
	resetValues();
	print("Done!!");
}
