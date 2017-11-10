#include "main.h"	//Include the main header file for any constants used in the program

void operatorControl() {
	while (true) {	//Run this program indefinitely
		driveOutputs[0] = joystickGetAnalog(1, JOYSTICK_DRIVE_S) + joystickGetAnalog(1, JOYSTICK_DRIVE_F);	//Store the drive’s left side values in driveOutputs[0]
		driveOutputs[1] = joystickGetAnalog(1, JOYSTICK_DRIVE_S) - joystickGetAnalog(1, JOYSTICK_DRIVE_F);	//Store the drive’s right side values in driveOutputs[1]
		
		//Set motor speeds based on calculated values
		motorSet(MOTOR_DRIVE_LB, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LF, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, driveOutputs[1]);

		delay(20);	//Delay 20 milliseconds so the processor doesn't overheat, but still can update the motors at a constant rate
	}
}
