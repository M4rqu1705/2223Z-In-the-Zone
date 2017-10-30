#include <drive.hpp>

namespace drive {
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
		if (powerOutput + SLEW_GAIN < joystickInputs[0]) powerOutput += SLEW_GAIN;
		else if (powerOutput - SLEW_GAIN > joystickInputs[0])	powerOutput -= SLEW_GAIN;

		if (turnOutput + SLEW_GAIN < joystickInputs[1]) turnOutput += SLEW_GAIN;
		else if (turnOutput - SLEW_GAIN > joystickInputs[1]) turnOutput -= SLEW_GAIN;

		//Calculate "arcade drive" values for left and right side
		if (directionNormal) {
			outputs[0] = ROUND(turnOutput) + ROUND(powerOutput);
			outputs[1] = ROUND(turnOutput) - ROUND(powerOutput);
		}
		else {
			outputs[0] = ROUND(turnOutput) - ROUND(powerOutput);
			outputs[1] = ROUND(turnOutput) + ROUND(powerOutput);
		}

		//Make sure left and right side values are within a range of values between -127 to 127
		outputs[0] = CLAMP(outputs[0]);
		outputs[1] = CLAMP(outputs[1]);

		//Move motors using calculated values for left and right side
		motorSet(MOTOR_DRIVE_LF, outputs[0]);
		motorSet(MOTOR_DRIVE_LB, outputs[0]);
		motorSet(MOTOR_DRIVE_RF, outputs[1]);
		motorSet(MOTOR_DRIVE_RB, outputs[1]);
	}

	void move(direction orientation, float pulses, int_fast8_t speed, bool useGyro) {
		//Recalculate pulses to convert them to degrees of rotation
		if (orientation == turnLeft || orientation == turnRight) {
			if (useGyro) pulses = DEGREES_ROTATION_TO_GYRO_TICKS(pulses);
			else pulses = DEGREES_ROTATION_TO_ENCODER_PULSES(pulses);
		}
		//Recaluclate pulses to convert them to inches of movement
		else if (orientation == forward || orientation == backward) pulses = INCHES_TRANSLATION_TO_ENCODER_PULSES(pulses);

		//Calculate PID and rectify robot if necessary
		if (useGyro) {
			if (orientation == forward || orientation == backward) {    //Calculate PID using encoder values and rectify with gyro
				PIDoutput = pid::PID(pid::PIDdrive, pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
				rectifyDriveGyro(outputs, PIDoutput, gyroGet(driveGyro));
			}
			else if (orientation == turnLeft || orientation == turnRight) {    //Calculate PID using gyro values and don't rectify
				PIDoutput = pid::PID(pid::PIDdrive, pulses, gyroGet(driveGyro));
				outputs[0] = PIDoutput;
				outputs[1] = PIDoutput;
			}
		}
		else {
			//Calculate PID using encoder values
			PIDoutput = pid::PID(pid::PIDdrive, pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
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
		if (PIDoutput < PID_DONE_THRESHOLD && PIDoutput > -PID_DONE_THRESHOLD) {
			if (counter < DRIVE_PID_CORRECTION_CYCLES) counter++;    //Sinchronous counter that doesn't affect other processes
			if (counter == DRIVE_PID_CORRECTION_CYCLES) {
				//If DRIVE_PID_CORRECTION_CYCLES time has passed since last time the robot was in position and it is still withing the threshold, it means that the drive was done
				if (useGyro) {
					if (orientation == forward || orientation == backward) {    //Calculate PID using encoder values and rectify with gyro
						PIDoutput = pid::PID(pid::PIDdrive, pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
					}
					else if (orientation == turnLeft || orientation == turnRight) {    //Calculate PID using gyro values and don't rectify
						PIDoutput = pid::PID(pid::PIDdrive, pulses, gyroGet(driveGyro));
					}
					else {
						PIDoutput = pid::PID(pid::PIDdrive, pulses, ((abs(encoderGet(encoderL)) + abs(encoderGet(encoderR))) / 2));
					}
					}
					if (PIDoutput < PID_DONE_THRESHOLD && PIDoutput > -PID_DONE_THRESHOLD) {
						notDone = false;
						if (useGyro) gyroReset(driveGyro);
					}
				}
				else notDone = true;
			}
			else counter = 0;
		}

	}
