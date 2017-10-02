#include "main.h"

//Claw
void clawControl(bool state) {
	signed char clawLoutput, clawRoutput;

	if (state) {
		clawLoutput = -127;
		clawRoutput = 127;
	}
	else {
		clawLoutput = 127;
		clawRoutput = -127;
	}

	if (clawCounter <= CLAW_CYCLES) {
		motorSet(MOTOR_CLAW_L, clawLoutput);
		motorSet(MOTOR_CLAW_R, clawRoutput);
	}
	else if (clawCounter > CLAW_CYCLES || clawCounter <= 0) {
		motorSet(MOTOR_CLAW_L, 0);
		motorSet(MOTOR_CLAW_R, 0);
	}
	clawCounter += (clawCounter <= CLAW_CYCLES + 1) ? 1 : 0;
}

//Mobile Goal
void mobileGoalControl(bool state) {
	signed char mogoLoutput, mogoRoutput;

	if (state) {
		mogoLoutput = -127;
		mogoRoutput = 127;
	}
	else {
		mogoLoutput = 127;
		mogoRoutput = -127;
	}

	if (mogoCounter <= MOGO_CYCLES) {
		motorSet(MOTOR_MOGO_L, mogoLoutput);
		motorSet(MOTOR_MOGO_R, mogoRoutput);
	}
	else if (mogoCounter > MOGO_CYCLES || mogoCounter <= 0) {
		motorSet(MOTOR_MOGO_L, 0);
		motorSet(MOTOR_MOGO_R, 0);
	}
	mogoCounter += (mogoCounter <= MOGO_CYCLES + 1) ? 1 : 0;
}

void operatorControl() {
	while (1) {

		//Drive

		if (joystickGetDigital(1, JOYSTICK_DRIVE_INVERT, JOYSTICK_DRIVE_INVERT_BUTTON) == 1) {
			if (!driveInvertButtonPressed) {
				driveInvertButtonPressed = true;
				driveDirectionNormal = !driveDirectionNormal;
			}
		}
		else driveInvertButtonPressed = false;

		joysDriveFvalue = joystickGetAnalog(1, JOYSTICK_DRIVE_F);
		joysDriveSvalue = joystickGetAnalog(1, JOYSTICK_DRIVE_S);

		joysDriveFvalue = (joysDriveFvalue >= DRIVE_THRESHOLD || joysDriveFvalue <= -DRIVE_THRESHOLD) ? joysDriveFvalue : 0;
		joysDriveSvalue = (joysDriveSvalue >= DRIVE_THRESHOLD || joysDriveSvalue <= -DRIVE_THRESHOLD) ? joysDriveSvalue : 0;

		if (driveFoutput + SLEW_GAIN < joysDriveFvalue) driveFoutput = driveDirectionNormal ? driveFoutput + SLEW_GAIN : driveFoutput - SLEW_GAIN;
		if (driveFoutput - SLEW_GAIN > joysDriveFvalue) driveFoutput = driveDirectionNormal ? driveFoutput - SLEW_GAIN : driveFoutput + SLEW_GAIN;
		if (driveSoutput + SLEW_GAIN < joysDriveSvalue) driveSoutput += SLEW_GAIN;
		if (driveSoutput - SLEW_GAIN > joysDriveSvalue) driveSoutput -= SLEW_GAIN;

		driveLoutput = driveFoutput + driveSoutput;
		driveRoutput = driveFoutput - driveSoutput;

		driveLoutput = abs(driveLoutput) > 127 ? driveLoutput < -127 ? -127 : 127 : 0;
		driveRoutput = abs(driveRoutput) > 127 ? driveRoutput < -127 ? -127 : 127 : 0;

		motorSet(MOTOR_DRIVE_LF, driveLoutput);
		motorSet(MOTOR_DRIVE_LB, driveLoutput);
		motorSet(MOTOR_DRIVE_RF, driveRoutput);
		motorSet(MOTOR_DRIVE_RB, driveRoutput);

		//Arm
		PIDarmL[3] = getSensor(filterArmL, analogRead(SENSOR_POT_L));
		PIDarmR[3] = getSensor(filterArmR, analogRead(SENSOR_POT_R));

		if (joystickGetDigital(1, JOYSTICK_ARM, JOYSTICK_ARM_BUTTON) == 1) {
			if (!armButtonPressed) {
				armButtonPressed = true;
				rightArmUp = !rightArmUp;
			}
		}
		else armButtonPressed = false;

		if (rightArmUp) {
			motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_UP));
			motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_DOWN));
		}
		else {
			motorSet(MOTOR_ARM_L, PID(PIDarmL, ARM_DOWN));
			motorSet(MOTOR_ARM_R, PID(PIDarmR, ARM_UP));
		}

		//Claw
		if (joystickGetDigital(1, JOYSTICK_CLAW, JOYSTICK_CLAW_BUTTON) == 1) {
			if (!clawButtonPressed) {
				clawButtonPressed = true;
				clawCounter = (clawCounter > CLAW_CYCLES || clawCounter <= 0) ? 1 : clawCounter;
				rightClawClosed = !rightClawClosed;
			}
		}
		else clawButtonPressed = false;

		clawControl(rightClawClosed);

		//Mobile Goal
		if (joystickGetDigital(1, JOYSTICK_MOGO, JOYSTICK_MOGO_BUTTON) == 1) {
			if (!mogoButtonPressed) {
				mogoButtonPressed = true;
				mogoCounter = (mogoCounter > MOGO_CYCLES || mogoCounter <= 0) ? 1 : mogoCounter;
				mogoRetracted = !mogoRetracted;
			}
		}
		else mogoButtonPressed = false;



		delay(LOOP_DELAY);
	}
}
