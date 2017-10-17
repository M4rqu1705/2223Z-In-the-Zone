#include "main.h"

/* If the robot is disabled or communications is lost, the autonomous task will be stopped by
 * the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off. */


void autonomous() {
	#if USING_LCD
	lcdShutdown(LCD_PORT);
	#endif

	resetValues();

	while (!driveDone || !armsDone || !clawsDone || !mogoDone) {
		drive(direction::f, 12, 100);	//Move 12 inches forward
		armsControl(armsPositions::u);	//Raise right arm and lower left
		clawsControl(1);	//Open right claw and close left
		mobileGoalControl(0);	//Extend mobile goal intake
	}
	resetValues();

	while (!driveDone || !armsDone || !clawsDone || !mogoDone) {
		drive(direction::b, 12, 100);	//Move 12 inches backward
		armsControl(armsPositions::lr);	//Lower right arm to loader level and raise left
		clawsControl(0);	//Close right claw and open left
		mobileGoalControl(1);	//Retract mobile goal intake
	}
	resetValues();

}


static void resetValues() {
	motorStopAll();

	// Variables

	#if USING_KALMAN_FILTER
	//Drive
	filterDrive[0] = DRIVE_FILTERS_KG_PRESET;	filterDrive[1] = DRIVE_FILTERS_ESTIMATE_PRESET;
	filterDrive[2] = DRIVE_FILTERS_PREVIOUS_ESTIMATE_PRESET;	filterDrive[3] = DRIVE_FILTERS_ERROR_ESTIMATE_PRESET;
	filterDrive[4] = DRIVE_FILTERS_PREVIOUS_ERROR_ESTIMATE_PRESET;	filterDrive[5] = DRIVE_FILTERS_ERROR_MEASUREMENT_PRESET;

	filterDriveL[0] = DRIVE_FILTERS_KG_PRESET;	filterDriveL[1] = DRIVE_FILTERS_ESTIMATE_PRESET;
	filterDriveL[2] = DRIVE_FILTERS_PREVIOUS_ESTIMATE_PRESET;	filterDriveL[3] = DRIVE_FILTERS_ERROR_ESTIMATE_PRESET;
	filterDriveL[4] = DRIVE_FILTERS_PREVIOUS_ERROR_ESTIMATE_PRESET;	filterDriveL[5] = DRIVE_FILTERS_ERROR_MEASUREMENT_PRESET;

	filterDriveR[0] = DRIVE_FILTERS_KG_PRESET;	filterDriveR[1] = DRIVE_FILTERS_ESTIMATE_PRESET;
	filterDriveR[2] = DRIVE_FILTERS_PREVIOUS_ESTIMATE_PRESET;	filterDriveR[3] = DRIVE_FILTERS_ERROR_ESTIMATE_PRESET;
	filterDriveR[4] = DRIVE_FILTERS_PREVIOUS_ERROR_ESTIMATE_PRESET;	filterDriveR[5] = DRIVE_FILTERS_ERROR_MEASUREMENT_PRESET;

	estimate = 0;

	//Arm
	filterArmL[0] = ARM_FILTERS_KG_PRESET;	filterArmL[1] = ARM_FILTERS_ESTIMATE_PRESET;
	filterArmL[2] = ARM_FILTERS_PREVIOUS_ESTIMATE_PRESET;	filterArmL[3] = ARM_FILTERS_ERROR_ESTIMATE_PRESET;
	filterArmL[4] = ARM_FILTERS_PREVIOUS_ERROR_ESTIMATE_PRESET;	filterArmL[5] = ARM_FILTERS_ERROR_MEASUREMENT_PRESET;

	filterArmR[0] = ARM_FILTERS_KG_PRESET;	filterArmR[1] = ARM_FILTERS_ESTIMATE_PRESET;
	filterArmR[2] = ARM_FILTERS_PREVIOUS_ESTIMATE_PRESET;	filterArmR[3] = ARM_FILTERS_ERROR_ESTIMATE_PRESET;
	filterArmR[4] = ARM_FILTERS_PREVIOUS_ERROR_ESTIMATE_PRESET;	filterArmR[5] = ARM_FILTERS_ERROR_MEASUREMENT_PRESET;
	#endif

	// Drive
	drivePowerOutput = 0;
	driveTurnOutput = 0;
	joystickDriveInputs[0] = 0;
	joystickDriveInputs[1] = 0;
	driveOutputs[0] = 0;
	driveOutputs[1] = 0;
	driveInvertButtonPressed = false;
	driveDirectionNormal = true;

	encoderReset(encoderLeft);
	encoderReset(encoderRight);

	PIDdrive[0] = DRIVE_PID_KP_PRESET;	PIDdrive[1] = DRIVE_PID_KI_PRESET; PIDdrive[2] = DRIVE_PID_KD_PRESET;

	PIDdrive[4] = DRIVE_PID_ERROR_PRESET;	PIDdrive[5] = DRIVE_PID_INTEGRAL_PRESET; PIDdrive[6] = DRIVE_PID_INTEGRAL_LIMIT_PRESET;
	PIDdrive[7] = DRIVE_PID_LAST_ERROR_PRESET;

	driveDone = false;

	// Arm
	PIDarmL[0] = ARM_PID_KP_PRESET;	PIDarmL[1] = ARM_PID_KI_PRESET; PIDarmL[2] = ARM_PID_KD_PRESET;
	PIDarmL[4] = ARM_PID_ERROR_PRESET;	PIDarmL[5] = ARM_PID_INTEGRAL_PRESET; PIDarmL[6] = ARM_PID_INTEGRAL_LIMIT_PRESET;
	PIDarmL[7] = ARM_PID_LAST_ERROR_PRESET;

	PIDarmR[0] = ARM_PID_KP_PRESET;	PIDarmR[1] = ARM_PID_KI_PRESET; PIDarmR[2] = ARM_PID_KD_PRESET;
	PIDarmR[4] = ARM_PID_ERROR_PRESET;	PIDarmR[5] = ARM_PID_INTEGRAL_PRESET; PIDarmR[6] = ARM_PID_INTEGRAL_LIMIT_PRESET;
	PIDarmR[7] = ARM_PID_LAST_ERROR_PRESET;

	armsButtonPressed = false;
	armsLoaderButtonPressed = false;
	currentArmPosition = armsPositions::u;

	armsDone = false;

	// Claw
	clawButtonPressed = false;
	rightClawClosed = true;
	clawsCounter = 1;

	clawsDone = false;

	//Mobile Goal
	mogoButtonPressed = false;
	mogoRetracted = true;
	mogoCounter = 1;

	mogoDone = false;
}
