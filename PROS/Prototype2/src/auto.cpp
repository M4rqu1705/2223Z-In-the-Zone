/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.hpp"

/*
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */
void autonomous() {
}

namespace drive{
	void move(direction orientation, float pulses, signed char speed, bool useGyro) {
		//Recalculate pulses to convert them to degrees of rotation
		if (orientation == turnLeft || orientation == turnRight) {
			if (useGyro) pulses = DEGREES_ROTATION_TO_GYRO_TICKS(pulses);
			else pulses = DEGREES_ROTATION_TO_ENCODER_PULSES(pulses);
		}
		//Recaluclate pulses to convert them to inches of movement
		else if (orientation == forward || orientation == backward){
			 pulses = INCHES_TRANSLATION_TO_ENCODER_PULSES(pulses);
		 }

		//Calculate PID and rectify robot if necessary
		if (useGyro) {
			if (orientation == forward || orientation == backward) {    //Calculate PID using encoder values and rectify with gyro
				PIDoutput = PID::calculatePID(PID::PIDdrive, pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
				rectifyDriveGyro(outputs, PIDoutput, gyroGet(driveGyro));
			}
			else if (orientation == turnLeft || orientation == turnRight) {    //Calculate PID using gyro values and don't rectify
				PIDoutput = PID::calculatePID(PID::PIDdrive, pulses, gyroGet(driveGyro));
				outputs[0] = PIDoutput;
				outputs[1] = PIDoutput;
			}
		}
		else {
			//Calculate PID using encoder values
			PIDoutput = PID::calculatePID(PID::PIDdrive, pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
			//Rectify with encoders only if moving forward or backwards
			if (orientation == forward || orientation == backward) rectifyOutputsEncoder(outputs, PIDoutput, abs(encoderGet(encoderL)), abs(encoderGet(encoderR)));
			else {
				outputs[0] = PIDoutput;
				outputs[1] = PIDoutput;
			}
		}

		//PIDoutput = PID(PIDdrive, pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
		//outputs[0] = PIDoutput;
		//outputs[1] = PIDoutput;

		//Make sure left and right orientation values are within a range of values between -speed to speed
		outputs[0] = MAP(outputs[0], -127, 127, -speed, speed);
		outputs[1] = MAP(outputs[1], -127, 127, -speed, speed);

		//Move motors based on PID values, direction in which to move
		switch (orientation) {
		case forward:
		motorSet(MOTOR_DRIVE_LF, outputs[0]);
		motorSet(MOTOR_DRIVE_LB, outputs[0]);
		motorSet(MOTOR_DRIVE_RF, -outputs[1]);
		motorSet(MOTOR_DRIVE_RB, -outputs[1]);
			break;

		case backward:
		motorSet(MOTOR_DRIVE_LF, -outputs[0]);
		motorSet(MOTOR_DRIVE_LB, outputs[0]);
		motorSet(MOTOR_DRIVE_RF, outputs[1]);
		motorSet(MOTOR_DRIVE_RB, outputs[1]);
			break;

		case turnLeft:
		motorSet(MOTOR_DRIVE_LF, -outputs[0]);
		motorSet(MOTOR_DRIVE_LB, -outputs[0]);
		motorSet(MOTOR_DRIVE_RF, -outputs[1]);
		motorSet(MOTOR_DRIVE_RB, -outputs[1]);
			break;

		case turnRight:
		motorSet(MOTOR_DRIVE_LF, outputs[0]);
		motorSet(MOTOR_DRIVE_LB, outputs[0]);
		motorSet(MOTOR_DRIVE_RF, outputs[1]);
		motorSet(MOTOR_DRIVE_RB, outputs[1]);
			break;

		}

		//Check if drive is done
		if (WITHIN_THRESHOLD(PID_DONE_THRESHOLD, PIDoutput, -PID_DONE_THRESHOLD)) {
			if (counter < DRIVE_PID_CORRECTION_CYCLES) counter++;    //Sinchronous counter that doesn't affect other processes
			if (counter == DRIVE_PID_CORRECTION_CYCLES) {
				//If DRIVE_PID_CORRECTION_CYCLES time has passed since last time the robot was in position and it is still withing the threshold, it means that the drive was done
				PIDoutput = PID::calculatePID(PID::PIDdrive, pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
				if (useGyro && (orientation == turnLeft || orientation == turnRight)) {    //Calculate PID using gyro values and don't rectify
						PIDoutput = PID::calculatePID(PID::PIDdrive, pulses, gyroGet(driveGyro));
				}
				if (PIDoutput < PID_DONE_THRESHOLD && PIDoutput > -PID_DONE_THRESHOLD) notDone = false;
				else notDone = true;
				counter = 0;
			}
			else{
				notDone = true;
			}
		}
	}
}
