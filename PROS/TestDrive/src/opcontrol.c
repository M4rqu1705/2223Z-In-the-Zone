#include "main.h"

void operatorControl() {
	while (true) {
		driveOutputs[0] = joystickGetAnalog(1, JOYSTICK_DRIVE_S) + joystickGetAnalog(1, JOYSTICK_DRIVE_F);
		driveOutputs[1] = joystickGetAnalog(1, JOYSTICK_DRIVE_S) - joystickGetAnalog(1, JOYSTICK_DRIVE_F);

		motorSet(MOTOR_DRIVE_LB, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LF, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, driveOutputs[1]);

		delay(20);
	}
}
