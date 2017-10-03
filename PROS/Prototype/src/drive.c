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
	joystickDriveInputs[0] = (joystickDriveInputs[0] >= DRIVE_THRESHOLD || joystickDriveInputs[0] <= -DRIVE_THRESHOLD) ? joystickDriveInputs[0] : 0;
	joystickDriveInputs[1] = (joystickDriveInputs[1] >= DRIVE_THRESHOLD || joystickDriveInputs[1] <= -DRIVE_THRESHOLD) ? joystickDriveInputs[1] : 0;

	//Slewrate control. Assign value to output incrementing or decreasing values using SLEW_GAIN
	//If direction inverted, decrease values where normally increased and vice versa
	if (drivePowerOutput + SLEW_GAIN < joystickDriveInputs[0]) drivePowerOutput = driveDirectionNormal ? drivePowerOutput + SLEW_GAIN : drivePowerOutput - SLEW_GAIN;
	if (drivePowerOutput - SLEW_GAIN > joystickDriveInputs[0]) drivePowerOutput = driveDirectionNormal ? drivePowerOutput - SLEW_GAIN : drivePowerOutput + SLEW_GAIN;
	if (driveTurnOutput + SLEW_GAIN < joystickDriveInputs[1]) driveTurnOutput += SLEW_GAIN;
	if (driveTurnOutput - SLEW_GAIN > joystickDriveInputs[1]) driveTurnOutput -= SLEW_GAIN;

	//Calculate "arcade drive" values for left and right side
	driveOutputs[0] = drivePowerOutput + driveTurnOutput;
	driveOutputs[1] = drivePowerOutput - driveTurnOutput;

	//Make sure left and right side values are within a range of values between -127 to 127
	driveOutputs[0] = withinRange(driveOutputs[0]);
	driveOutputs[1] = withinRange(driveOutputs[1]);

	//Move motors using calculated values for left and right side
	motorSet(MOTOR_DRIVE_LF, driveOutputs[0]);
	motorSet(MOTOR_DRIVE_LB, driveOutputs[0]);
	motorSet(MOTOR_DRIVE_RF, driveOutputs[1]);
	motorSet(MOTOR_DRIVE_RB, driveOutputs[1]);
}


void drive(direction orientation, unsigned short pulses, signed char speed, bool useGyro) {

	//Update current sensorValue using filters
	PIDdrive[3] = getSensor(filterDrive, (abs(encoderGet(encoderLeft)) + abs(encoderGet(encoderRight)) / 2));

	//Recalculate pulses to convert them to degrees of rotation
	if (orientation == l || orientation == r) pulses = useGyro ? degreesOfRotationToGyroTicks(pulses) : degreesOfRotationToEncoderPulses(pulses);

	//Recaluclate pulses to convert them to inches of movement
	else pulses = inchesOfTranslationToEncoderPulses(pulses);

	//Calculate PID
	PIDoutput = PID(PIDdrive, pulses, true, true);

	//Rectify robot if necessary
	rectifyOutputs(driveOutputs, PIDoutput, getSensor(filterDriveL, abs(encoderGet(encoderLeft))), getSensor(filterDriveR, abs(encoderGet(encoderRight))));

	//Make sure left and right orientation values are within a range of values between -speed to speed
	driveOutputs[0] = map(withinRange(driveOutputs[0]), -127, 127, -speed, speed);
	driveOutputs[1] = map(withinRange(driveOutputs[1]), -127, 127, -speed, speed);

	//Move motors based on PID values, direction in which to move
	switch (orientation) {
	case f:
		motorSet(MOTOR_DRIVE_LF, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LB, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, driveOutputs[1]);
		break;

	case b:
		motorSet(MOTOR_DRIVE_LF, -driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LB, -driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, -driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, -driveOutputs[1]);
		break;

	case l:
		motorSet(MOTOR_DRIVE_LF, -driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LB, -driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, driveOutputs[1]);
		break;

	case r:
		motorSet(MOTOR_DRIVE_LF, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_LB, driveOutputs[0]);
		motorSet(MOTOR_DRIVE_RF, -driveOutputs[1]);
		motorSet(MOTOR_DRIVE_RB, -driveOutputs[1]);
		break;
		break;

	}

	driveDone = (PIDoutput <= PID_DONE_THRESHOLD && PIDoutput >= -PID_DONE_THRESHOLD) ? true : false;
}
