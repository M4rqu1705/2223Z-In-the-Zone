#include <main.hpp>

namespace arms {
	void changePositionTo(position state) {
		//Based on state of variable 'state', set motors to different values

		switch (state) {
		case rightUp:
			//Move motors based on pid::PID calculated values
			motorSet(MOTOR_ARM_L, CLAMP(pid::PID(pid::PIDarmL, ARM_DOWN, analogRead(SENSOR_POT_L))));
			motorSet(MOTOR_ARM_R, CLAMP(pid::PID(pid::PIDarmR, ARM_UP, analogRead(SENSOR_POT_R))));

			//Check if arm is in position within the threshold
			if (pid::PID(pid::PIDarmR, ARM_UP, analogRead(SENSOR_POT_R)) < PID_DONE_THRESHOLD && pid::PID(pid::PIDarmR, ARM_UP, analogRead(SENSOR_POT_R)) > -PID_DONE_THRESHOLD) notDone = false;
			else notDone = true;
			break;

		case rightDown:
			//Move motors based on pid::PID calculated values
			motorSet(MOTOR_ARM_L, CLAMP(pid::PID(pid::PIDarmL, ARM_UP, analogRead(SENSOR_POT_L))));
			motorSet(MOTOR_ARM_R, CLAMP(pid::PID(pid::PIDarmR, ARM_DOWN, analogRead(SENSOR_POT_R))));

			//Check if arm is in position within the threshold
			if (pid::PID(pid::PIDarmL, ARM_UP, analogRead(SENSOR_POT_L)) < PID_DONE_THRESHOLD && pid::PID(pid::PIDarmL, ARM_UP, analogRead(SENSOR_POT_L)) > -PID_DONE_THRESHOLD) notDone = false;
			else notDone = true;
			break;

		case rightLoader:
			//Move motors based on pid::PID calculated values
			motorSet(MOTOR_ARM_L, CLAMP(pid::PID(pid::PIDarmL, ARM_UP, analogRead(SENSOR_POT_L))));
			motorSet(MOTOR_ARM_R, CLAMP(pid::PID(pid::PIDarmR, ARM_LOADER, analogRead(SENSOR_POT_R))));

			//Check if arm is in position within the threshold
			if (pid::PID(pid::PIDarmL, ARM_UP, analogRead(SENSOR_POT_L)) < PID_DONE_THRESHOLD && pid::PID(pid::PIDarmL, ARM_UP, analogRead(SENSOR_POT_L)) > -PID_DONE_THRESHOLD) notDone = false;
			else notDone = true;
			break;

		case leftLoader:
			//Move motors based on pid::PID calculated values
			motorSet(MOTOR_ARM_L, CLAMP(pid::PID(pid::PIDarmL, ARM_LOADER, analogRead(SENSOR_POT_L))));
			motorSet(MOTOR_ARM_R, CLAMP(pid::PID(pid::PIDarmR, ARM_UP, analogRead(SENSOR_POT_R))));

			//Check if arm is in position within the threshold
			if (pid::PID(pid::PIDarmR, ARM_UP, analogRead(SENSOR_POT_R)) < PID_DONE_THRESHOLD && pid::PID(pid::PIDarmR, ARM_UP, analogRead(SENSOR_POT_R)) > -PID_DONE_THRESHOLD) notDone = false;
			else notDone = true;
			break;
		}
	}

	void operatorControl() {
		//Button toggle for "normal button"
		if (joystickGetDigital(1, JOYSTICK_ARM, JOYSTICK_ARM_BUTTON) == 1) {
			if (!buttonPressed) {
				buttonPressed = true;
				if (currentPosition == rightDown) currentPosition = rightUp;
				else currentPosition = rightDown;
			}
		}
		else buttonPressed = false;

		//Button toggle for "loader button"
		if (joystickGetDigital(1, JOYSTICK_ARM_LOADER, JOYSTICK_ARM_LOADER_BUTTON) == 1) {
			if (!loaderButtonPressed) {
				loaderButtonPressed = true;
				switch (currentPosition) {
				case rightUp:
					currentPosition = rightLoader;
					break;
				case rightDown:
					currentPosition = leftLoader;
					break;
				case rightLoader:
					currentPosition = leftLoader;
					break;
				case leftLoader:
					currentPosition = rightLoader;
					break;
				}
			}
		}
		else loaderButtonPressed = false;

		//Based on state of variable 'currentPosition', set motors to different values
		changePositionTo(currentPosition);
	}
}
