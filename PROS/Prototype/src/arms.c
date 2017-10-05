#include "main.h"

void armsOperatorControl() {
	//Refresh sensorValues to be used in PID calculations
	PIDarmL[3] = getSensor(filterArmL, analogRead(SENSOR_POT_L));
	PIDarmR[3] = getSensor(filterArmR, analogRead(SENSOR_POT_R));

	//Button toggle
	if (joystickGetDigital(1, JOYSTICK_ARM, JOYSTICK_ARM_BUTTON) == 1) {
		if (!armsButtonPressed) {
			armsButtonPressed = true;
			currentArmPosition = (currentArmPosition == armsPositions::d) ? armsPositions::u : armsPositions::d;
		}
	}
	else armsButtonPressed = false;

	if (joystickGetDigital(1, JOYSTICK_ARM_LOADER, JOYSTICK_ARM_LOADER_BUTTON) == 1) {
		if (!armsLoaderButtonPressed) {
			armsLoaderButtonPressed = true;
			switch (currentArmPosition) {
			case armsPositions::u:
				currentArmPosition = armsPositions::lr;
				break;
			case armsPositions::d:
				currentArmPosition = armsPositions::ll;
				break;
			case armsPositions::lr:
				currentArmPosition = armsPositions::ll;
				break;
			case armsPositions::ll:
				currentArmPosition = armsPositions::lr;
				break;
			}
		}
	}
	else armsLoaderButtonPressed = false;

	//Based on state of variable 'currentArmPosition', set motors to different values
	armsControl(currentArmPosition);
}

void armsControl(armsPositions state) {
	//Refresh sensorValues to be used in PID calculations
	PIDarmL[3] = getSensor(filterArmL, analogRead(SENSOR_POT_L));
	PIDarmR[3] = getSensor(filterArmR, analogRead(SENSOR_POT_R));

	//Based on state of variable 'state', set motors to different values
	switch (state) {
	case armsPositions::u:
		motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_DOWN));
		motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_UP));
		if (PID(PIDarmR, ARM_UP) > PID_DONE_THRESHOLD || PID(PIDarmR, ARM_UP) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;
	case armsPositions::d:
		motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_UP));
		motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_DOWN));
		if (PID(PIDarmL, ARM_UP) > PID_DONE_THRESHOLD || PID(PIDarmL, ARM_UP) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;
	case armsPositions::lr:
		motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_UP));
		motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_LOADER));
		if (PID(PIDarmL, ARM_UP) > PID_DONE_THRESHOLD || PID(PIDarmL, ARM_UP) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;
	case armsPositions::ll:
		motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_LOADER));
		motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_UP));
		if (PID(PIDarmR, ARM_UP) > PID_DONE_THRESHOLD || PID(PIDarmR, ARM_UP) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;

	}
}
