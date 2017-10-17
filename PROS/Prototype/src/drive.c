#include "main.h"

void driveOperatorControl() {

	//Button toggle
	if (joystickGetDigital(1, JOYSTICK_DRIVE_INVERT, JOYSTICK_DRIVE_INVERT_BUTTON) == 1) {
		if (!driveInvertButtonPressed) {
			driveInvertButtonPressed = true;
			driveDirectionNormal = !driveDirectionNormal;
		}
	}
	else driveInvertButtonPressed = false;

	//Assign joystick values to variables
	joystickDriveInputs[0] = joystickGetAnalog(1, JOYSTICK_DRIVE_F);
	joystickDriveInputs[1] = joystickGetAnalog(1, JOYSTICK_DRIVE_S);

	//Only use values if withing threshold. If not withing threshold, assign 0
	if (joystickDriveInputs[0] < DRIVE_THRESHOLD || joystickDriveInputs[0] > -DRIVE_THRESHOLD) joystickDriveInputs[0] = 0;
	if (joystickDriveInputs[1] < DRIVE_THRESHOLD || joystickDriveInputs[1] > -DRIVE_THRESHOLD) joystickDriveInputs[1] = 0;

	//Slewrate control. Assign value to output incrementing or decreasing values using SLEW_GAIN
	//If direction inverted, decrease values where normally increased and vice versa
	if (drivePowerOutput + SLEW_GAIN < joystickDriveInputs[0]) {
		if (driveDirectionNormal) drivePowerOutput += ROUND(SLEW_GAIN);
		else drivePowerOutput -= ROUND(SLEW_GAIN);
	}
	else if (drivePowerOutput - SLEW_GAIN > joystickDriveInputs[0]) {
		if (driveDirectionNormal) drivePowerOutput -= ROUND(SLEW_GAIN);
		else drivePowerOutput += ROUND(SLEW_GAIN);
	}
	if (driveTurnOutput + SLEW_GAIN < joystickDriveInputs[1]) driveTurnOutput += SLEW_GAIN;
	if (driveTurnOutput - SLEW_GAIN > joystickDriveInputs[1]) driveTurnOutput -= SLEW_GAIN;

	//Calculate "arcade drive" values for left and right side
	driveOutputs[0] = drivePowerOutput + driveTurnOutput;
	driveOutputs[1] = drivePowerOutput - driveTurnOutput;

	//Make sure left and right side values are within a range of values between -127 to 127
	driveOutputs[0] = WITHIN_RANGE(driveOutputs[0]);
	driveOutputs[1] = WITHIN_RANGE(driveOutputs[1]);

	//Move motors using calculated values for left and right side
	motorSet(MOTOR_DRIVE_LF, driveOutputs[0]);
	motorSet(MOTOR_DRIVE_LB, driveOutputs[0]);
	motorSet(MOTOR_DRIVE_RF, driveOutputs[1]);
	motorSet(MOTOR_DRIVE_RB, driveOutputs[1]);
}


void drive(direction orientation, uint_fast16_t pulses, int_fast8_t speed, bool useGyro) {
	//Recalculate pulses to convert them to degrees of rotation
	if (orientation == direction::l || orientation == direction::r) {
		if (useGyro) pulses = DEGREES_ROTATION_TO_GYRO_TICKS(pulses);
		else pulses = DEGREES_ROTATION_TO_ENCODER_PULSES(pulses);
	}
	//Recaluclate pulses to convert them to inches of movement
	else pulses = INCHES_TRANSLATION_TO_ENCODER_PULSES(pulses);

	//Calculate PID and rectify robot if necessary
	#if USING_KALMAN_FILTER
	PIDoutput = PID(PIDdrive, pulses, getSensor(PIDdrive, pulses, (abs(encoderGet(encoderLeft)) + abs(encoderGet(encoderRight)) / 2));
	rectifyOutputs(driveOutputs, PIDoutput, getSensor(filterDriveL, abs(encoderGet(encoderLeft))), getSensor(filterDriveR, abs(encoderGet(encoderRight))));
	#else
	PIDoutput = PID(PIDdrive, pulses, (abs(encoderGet(encoderLeft)) + abs(encoderGet(encoderRight)) / 2);
	rectifyOutputs(driveOutputs, PIDoutput, abs(encoderGet(encoderLeft)), abs(encoderGet(encoderRight));
	#endif

	//Make sure left and right orientation values are within a range of values between -speed to speed
	driveOutputs[0] = MAP(WITHIN_RANGE(driveOutputs[0]), -127, 127, -speed, speed);
	driveOutputs[1] = MAP(WITHIN_RANGE(driveOutputs[1]), -127, 127, -speed, speed);

	//Move motors based on PID values, direction in which to move
	switch (orientation) {
	case direction::f:
		motorSet(MOTOR_DRIVE_LF, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LB, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, driveOutputs[1]);
		break;

	case direction::b:
		motorSet(MOTOR_DRIVE_LF, -driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LB, -driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, -driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, -driveOutputs[1]);
		break;

	case direction::l:
		motorSet(MOTOR_DRIVE_LF, -driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LB, -driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, driveOutputs[1]);
		break;

	case direction::r:
		motorSet(MOTOR_DRIVE_LF, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LB, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, -driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, -driveOutputs[1]);
		break;
		break;

	}
	if (PIDoutput > PID_DONE_THRESHOLD || PIDoutput < -PID_DONE_THRESHOLD) driveDone = false;
	else driveDone = true;
}
