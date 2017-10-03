#include "main.h"

void armsOperatorControl() {
	//Refresh sensorValues to be used in PID calculations
	PIDarmL[3] = getSensor(filterArmL, analogRead(SENSOR_POT_L));
	PIDarmR[3] = getSensor(filterArmR, analogRead(SENSOR_POT_R));

	//Button toggle
	if (joystickGetDigital(1, JOYSTICK_ARM, JOYSTICK_ARM_BUTTON) == 1) {
		if (!armsButtonPressed) {
			armsButtonPressed = true;
			armsPosition = armsPosition == d ? u : d;
		}
	}
	else armsButtonPressed = false;

	if (joystickGetDigital(1, JOYSTICK_ARM_LOADER, JOYSTICK_ARM_LOADER_BUTTON) == 1) {
		if (!armsLoaderButtonPressed) {
			armsLoaderButtonPressed = true;
			armsPosition = armsPosition == u ? lr : armsPosition == d ? ll : armsPosition == lr ? ll : lr;
		}
	}
	else armsLoaderButtonPressed = false;

	//Based on state of variable 'armsPosition', set motors to different values
	armsControl(armsPosition);
}

void armsControl(armsPositions state) {
	//Refresh sensorValues to be used in PID calculations
	PIDarmL[3] = getSensor(filterArmL, analogRead(SENSOR_POT_L));
	PIDarmR[3] = getSensor(filterArmR, analogRead(SENSOR_POT_R));

	//Based on state of variable 'state', set motors to different values
	if (state == u) {
		motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_DOWN));
		motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_UP));
		armsDone = (PID(PIDarmR, ARM_UP) <= PID_DONE_THRESHOLD && PID(PIDarmR, ARM_UP) >= -PID_DONE_THRESHOLD) ? true : false;
	}
	else if (state == d) {
		motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_UP));
		motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_DOWN));
		armsDone = (PID(PIDarmL, ARM_UP) <= PID_DONE_THRESHOLD && PID(PIDarmL, ARM_UP) >= -PID_DONE_THRESHOLD) ? true : false;
	}
	else if (state == lr) {
		motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_UP));
		motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_LOADER));
		armsDone = (PID(PIDarmL, ARM_UP) <= PID_DONE_THRESHOLD && PID(PIDarmL, ARM_UP) >= -PID_DONE_THRESHOLD) ? true : false;
	}
	else if (state == ll) {
		motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_LOADER));
		motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_UP));
		armsDone = (PID(PIDarmR, ARM_UP) <= PID_DONE_THRESHOLD && PID(PIDarmR, ARM_UP) >= -PID_DONE_THRESHOLD) ? true : false;
	}

}
