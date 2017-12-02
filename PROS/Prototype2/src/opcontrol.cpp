/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.hpp"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
	while (1) {
		delay(20);
	}
}

namespace drive{
	void operatorControl() {

		//Button toggle
		if (joystickGetDigital(1, JOYSTICK_DRIVE_INVERT, JOYSTICK_DRIVE_INVERT_BUTTON) == 1) {
			if (!invertButtonPressed) {
				invertButtonPressed = true;
				directionNormal = !directionNormal;
			}
		}
		else invertButtonPressed = false;

		//Assign joystick values to variables
		joystickInputs[0] = joystickGetAnalog(1, JOYSTICK_DRIVE_F);
		joystickInputs[1] = joystickGetAnalog(1, JOYSTICK_DRIVE_S);

		//Only use values if withing threshold. If not withing threshold, assign 0
		if (joystickInputs[0] < DRIVE_THRESHOLD && joystickInputs[0] > -DRIVE_THRESHOLD) joystickInputs[0] = 0;
		if (joystickInputs[1] < DRIVE_THRESHOLD && joystickInputs[1] > -DRIVE_THRESHOLD) joystickInputs[1] = 0;

		//Slewrate control. Assign value to output incrementing or decreasing values using SLEW_GAIN
		//If direction inverted, decrease values where normally increased and vice versa
		slewOutputs[0] += CLAMP(SLEW_GAIN * SIGN(joystickInputs[0], directionNormal? 1 : -1));
		slewOutputs[1] += CLAMP(SLEW_GAIN * SIGN(joystickInputs[1], directionNormal? 1 : -1));

		//Calculate "arcade drive" values for left and right side
		if (directionNormal) {
			outputs[0] = CLAMP(ROUND(slewOutputs[1]) + ROUND(slewOutputs[0]));
			outputs[1] = CLAMP(ROUND(slewOutputs[1]) - ROUND(slewOutputs[0]));
		}
		else {
			outputs[0] = CLAMP(ROUND(slewOutputs[1]) - ROUND(slewOutputs[0]));
			outputs[1] = CLAMP(ROUND(slewOutputs[1]) + ROUND(slewOutputs[0]));
		}

		//Move motors using calculated values for left and right side
		motorSet(MOTOR_DRIVE_LF, outputs[0]);
		motorSet(MOTOR_DRIVE_LB, outputs[0]);
		motorSet(MOTOR_DRIVE_RF, outputs[1]);
		motorSet(MOTOR_DRIVE_RB, outputs[1]);
	}
}