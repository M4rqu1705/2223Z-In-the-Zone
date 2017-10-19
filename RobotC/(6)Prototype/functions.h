#ifndef functions.h
#define functions.h

#pragma systemfile

//**======================================================**Previously on .h files**======================================================**//

//Constants
//#include "constants.h"

//Conversions
float temp;

//PID
float PIDdrive[8], PIDarmL[8], PIDarmR[8];
int output;

//Kalman filters
float filterDrive[6], filterDriveL[6], filterDriveR[6], filterArmL[6], filterArmR[6];
int estimate;

//Drive
short drivePowerOutput, driveTurnOutput, PIDoutput;
int driveOutputs[2], joystickDriveInputs[2];
bool driveInvertButtonPressed, driveDirectionNormal, driveDone;

enum direction { f = 0, b = 1, l = 2, r = 3 };

//Arms
bool armsButtonPressed, armsLoaderButtonPressed, armsDone;
enum armsPositions { d = 0, u = 1, lr = 2, ll = 3 };
armsPositions currentArmPosition = d;

//Claws
bool clawButtonPressed, rightClawClosed, clawsDone;
unsigned short clawsCounter;
short clawLoutput, clawRoutput;

//Mobile Goal Intake
bool mogoButtonPressed, mogoRetracted, mogoDone;
unsigned short mogoCounter;
short mogoLoutput, mogoRoutput;

//LCD
enum lcdMenus { mainMenu = 0, batteryVoltageMenu = 1, backupBatteryVoltageMenu = 2, autonomousMenu = 3, allianceColorMenu = 4, startingSideMenu = 5 };
lcdMenus currentMenu = mainMenu;

enum autonomousCodes { mogoAndCones = 0, mogo = 1, cones = 2 };
autonomousCodes currentCode = mogoAndCones;
enum allianceColor { red = 0, blue = 1 };
allianceColor currentColor = red;
enum startingSide { leftSide = 0, rightSide = 1 };
startingSide currentSide = rightSide;

string lcdOutput;
bool lcdButtonsPressed[3] = { false, false, false }, lcdReady;

void lcdSelect();


//**======================================================**Previously on .c files**======================================================**//

int ROUND_NUMBER(float inNumber){
	temp = ceil(inNumber - 0.49);
	return temp;
}
//http://www.cplusplus.com/forum/beginner/3600/

int MAP(int inNumber, int inMin, int inMax, int outMin, int outMax) {
	return (inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

byte WITHIN_RANGE(short inNumber) {
	return abs(inNumber) > 127 ? inNumber < -127 ? -127 : 127 : 0;
}

int INCHES_TRANSLATION_TO_ENCODER_PULSES(int inches) {
	return (WHEEL_DIAMETER*PI)*(inches / 360);
}

int DEGREES_ROTATION_TO_GYRO_TICKS(int targetDegrees) {
	temp = (targetDegrees *(GYRO_FULL_ROTATION_TICKS / 360));
	return ROUND_NUMBER(temp);
}

int DEGREES_ROTATION_TO_ENCODER_PULSES(int targetDegrees) {
	temp = ((targetDegrees / 360) * DRIVE_WIDTH * PI) / (WHEEL_DIAMETER / 360);
	return ROUND_NUMBER(temp);
}

void rectifyOutputs(int *values, int speed, int leftSideSensor, int rightSideSensor) {
	values[0] = (speed / (leftSideSensor - rightSideSensor)) * 0.01 + 1;
	values[1] = (speed / (rightSideSensor - leftSideSensor)) * 0.01 + 1;
}

//Reset Values
static void resetValues() {
	motor[1] = motor[2] = motor[3] = motor[4] = motor[5] = motor[6] = motor[7] = motor[8] = motor[9] = motor[10] = 0;

	// Variables

#ifdef USING_KALMAN_FILTER
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

	SensorValue[encoderL] = 0;
	SensorValue[encoderR]  = 0;

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
	armsDone = false;

	// Claw
	clawButtonPressed = false;
	clawsCounter = 1;
	clawsDone = false;

	//Mobile Goal
	mogoButtonPressed = false;
	mogoCounter = 1;
	mogoDone = false;
}

//Initialize

void initialize(){
#ifdef USING_LCD

	clearLCDLine(0);
	clearLCDLine(1);
	bLCDBacklight = LCD_BACKLIGHT;

	currentMenu = mainMenu;
	currentCode = mogoAndCones;
	currentColor = red;
	currentSide = rightSide;
	lcdButtonsPressed[0] = false;
	lcdButtonsPressed[1] = false;
	lcdButtonsPressed[2] = false;

	if (bIfiRobotDisabled){
		lcdReady = false;
		displayLCDCenteredString(0, "Calibrating Gyro");
		displayLCDCenteredString(1, "...");
	}
	else {
		lcdReady = true;
	}
#endif
	if(bIfiRobotDisabled){
	//Completely clear out any previous sensor readings by setting the port to "sensorNone"
	SensorType[gyro] = sensorNone;
	wait1Msec(1000);
	//Reconfigure Analog Port 8 as a Gyro sensor and allow time for ROBOTC to calibrate it
	SensorType[gyro] = sensorGyro;
	wait1Msec(1500);

	//Adjust SensorScale to correct the scaling for your gyro
	SensorScale[in8] = 260;
	//Adjust SensorFullCount to set the "rollover" point. A value of 3600 sets the rollover point to +/-3600
	SensorFullCount[in8] = 3600;
	}

	resetValues();

#ifdef USING_LCD
	do{
		if(bIfiRobotDisabled)	lcdSelect();
		else break;
	}while (!lcdReady);

	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDCenteredString(0, "     2223-Z     ");
#endif
}

//PID
short PID(float *values, int target, unsigned int sensorInput) {
	/*
	values[] array format:
	KP(0),  KI(1), KD(2), , error(3), integral(4), integralLimit(5), lastError(6)
	*/

	values[3] = target - sensorInput;
	values[4] += (abs(values[4] + values[3]) < values[5]) ? values[3] : 0;
	if (values[3] == 0) values[4] = 0;

	output = values[0] * values[3] + values[1] * values[4] + values[2] * (values[6] - values[3]);

	values[6] = values[3];

	return WITHIN_RANGE(output);
}

//Kalman filter
int getSensor(float *values, unsigned int sensorInput) {
	/*	values[] array format:
	KG(0), estimate(1), previousEstimate(2), errorEstimate(3), previousErrorEstimate(4), errorMeasurement(5) */

	//Calculate Kalman Gain
	if (values[3] - values[5] != 0) values[0] = values[3] / (values[3] - values[5]);
	else values[0] = 0;

	//Calculate Estimate
	values[1] = sensorInput + values[0] * (values[2]-sensorInput);

	//Calculate Error in Estimate
	values[3] = (1 - values[0]) * values[4];

	values[2] = values[1]; values[4] = values[3];

	estimate = ROUND_NUMBER(values[1]); //Convert float to int

	return estimate;
}

//Drive
void driveOperatorControl() {

	//Button toggle
	if (vexRT[JOYSTICK_DRIVE_INVERT] == 1) {
		if (!driveInvertButtonPressed) {
			driveInvertButtonPressed = true;
			driveDirectionNormal = !driveDirectionNormal;
		}
	}
	else driveInvertButtonPressed = false;

	//Assign joystick values to variables
	joystickDriveInputs[0] = vexRT[JOYSTICK_DRIVE_F];
	joystickDriveInputs[1] = vexRT[JOYSTICK_DRIVE_S];

	//Only use values if withing threshold. If not withing threshold, assign 0
	if (joystickDriveInputs[0] < DRIVE_THRESHOLD || joystickDriveInputs[0] > -DRIVE_THRESHOLD) joystickDriveInputs[0] = 0;
	if (joystickDriveInputs[1] < DRIVE_THRESHOLD || joystickDriveInputs[1] > -DRIVE_THRESHOLD) joystickDriveInputs[1] = 0;

	//Slewrate control. Assign value to output incrementing or decreasing values using SLEW_GAIN
	//If direction inverted, decrease values where normally increased and vice versa
	if (drivePowerOutput + SLEW_GAIN < joystickDriveInputs[0]) {
		if (driveDirectionNormal) drivePowerOutput += ROUND_NUMBER(SLEW_GAIN);
		else drivePowerOutput -= ROUND_NUMBER(SLEW_GAIN);
	}
	else if (drivePowerOutput - SLEW_GAIN > joystickDriveInputs[0]) {
		if (driveDirectionNormal) drivePowerOutput -= ROUND_NUMBER(SLEW_GAIN);
		else drivePowerOutput += ROUND_NUMBER(SLEW_GAIN);
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
	motor[driveLF] = motor[driveLB] = driveOutputs[0];
	motor[driveRF] = motor[driveRB] = driveOutputs[1];
}


void drive(direction orientation, int pulses, short speed, bool useGyro = true) {
	//Recalculate pulses to convert them to degrees of rotation
	if (orientation == l || orientation == r) {
		if (useGyro) pulses = DEGREES_ROTATION_TO_GYRO_TICKS(pulses);
		else pulses = DEGREES_ROTATION_TO_ENCODER_PULSES(pulses);
	}
	//Recaluclate pulses to convert them to inches of movement
	else pulses = INCHES_TRANSLATION_TO_ENCODER_PULSES(pulses);

	//Calculate PID and rectify robot if necessary
#ifdef USING_KALMAN_FILTER
	temp = (abs(SensorValue[encoderL]) + abs(SensorValue[encoderR])) / 2;
	PIDoutput = PID(PIDdrive, pulses, getSensor(filterDrive, temp));
	rectifyOutputs(driveOutputs, PIDoutput, getSensor(filterDriveL, abs(SensorValue[encoderL])), getSensor(filterDriveR, abs(SensorValue[encoderR])));
#else
	PIDoutput = PID(PIDdrive, pulses, ((abs(SensorValue[encoderL]) + abs(SensorValue[encoderR])) / 2));
	rectifyOutputs(driveOutputs, PIDoutput, abs(SensorValue[encoderL]), abs(SensorValue[encoderR]));
#endif

	//Make sure left and right orientation values are within a range of values between -speed to speed
	driveOutputs[0] = MAP(WITHIN_RANGE(driveOutputs[0]), -127, 127, -speed, speed);
	driveOutputs[1] = MAP(WITHIN_RANGE(driveOutputs[1]), -127, 127, -speed, speed);

	//Move motors based on PID values, direction in which to move
	switch (orientation) {
	case f:
		motor[driveLF] = motor[driveLB] = driveOutputs[0];
		motor[driveRF] = motor[driveRB] = driveOutputs[1];
		break;

	case b:
		motor[driveLF] = motor[driveLB] = -driveOutputs[0];
		motor[driveRF] = motor[driveRB] = -driveOutputs[1];
		break;

	case l:
		motor[driveLF] = motor[driveLB] = -driveOutputs[0];
		motor[driveRF] = motor[driveRB] = driveOutputs[1];
		break;

	case r:
		motor[driveLF] = motor[driveLB] = driveOutputs[0];
		motor[driveRF] = motor[driveRB] = -driveOutputs[1];
		break;

	}
	if (PIDoutput > PID_DONE_THRESHOLD || PIDoutput < -PID_DONE_THRESHOLD) driveDone = false;
	else driveDone = true;
}

//Arms
void armsControl(armsPositions state) {
	//Based on state of variable 'state', set motors to different values

#ifdef USING_KALMAN_FILTER
	switch (state) {
	case u:
		motor[armL] = PID(PIDarmL, ARM_DOWN, getSensor(filterArmL, SensorValue[potL]));
		motor [armR] = PID(PIDarmR, ARM_UP, getSensor(filterArmR, SensorValue[potR]));
		if (PID(PIDarmR, ARM_UP, getSensor(filterArmR, SensorValue[potR])) > PID_DONE_THRESHOLD || PID(PIDarmR, ARM_UP, getSensor(filterArmR, SensorValue[potR])) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;

	case d:
		motor [armL] = PID(PIDarmL, ARM_UP, getSensor(filterArmL, SensorValue[potL]));
		motor[armR] = PID(PIDarmR, ARM_DOWN, getSensor(filterArmR, SensorValue[potR]));
		if (PID(PIDarmL, ARM_UP, getSensor(filterArmL, SensorValue[potL])) > PID_DONE_THRESHOLD || PID(PIDarmL, ARM_UP, getSensor(filterArmL, SensorValue[potL])) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;

	case lr:
		motor[armL] = PID(PIDarmL, ARM_UP, getSensor(filterArmL, SensorValue[potL]));
		motor[armR] = PID(PIDarmR, ARM_LOADER, getSensor(filterArmR, SensorValue[potR]));
		if (PID(PIDarmL, ARM_UP, getSensor(filterArmL, SensorValue[potL])) > PID_DONE_THRESHOLD || PID(PIDarmL, ARM_UP, getSensor(filterArmL, SensorValue[potL])) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;

	case ll:
		motor[armL] = PID(PIDarmL, ARM_LOADER, getSensor(filterArmL, SensorValue[potL]));
		motor[armR] = PID(PIDarmR, ARM_UP, getSensor(filterArmR, SensorValue[potR]));
		if (PID(PIDarmR, ARM_UP, getSensor(filterArmR, SensorValue[potR])) > PID_DONE_THRESHOLD || PID(PIDarmR, ARM_UP, getSensor(filterArmR, SensorValue[potR])) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;
	}
#else
	switch (state) {
	case u:
		motor[armL] = PID(PIDarmL, ARM_DOWN, SensorValue[potL]);
		motor[armR] = PID(PIDarmR, ARM_UP, SensorValue[potR]);
		if (PID(PIDarmR, ARM_UP, SensorValue[potR]) > PID_DONE_THRESHOLD || PID(PIDarmR, ARM_UP, SensorValue[potR]) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;

	case d:
		motor[armL] = PID(PIDarmL, ARM_UP, SensorValue[potL]);
		motor[armR] = PID(PIDarmR, ARM_DOWN, SensorValue[potR]);
		if (PID(PIDarmL, ARM_UP, SensorValue[potL]) > PID_DONE_THRESHOLD || PID(PIDarmL, ARM_UP, SensorValue[potL]) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;

	case lr:
		motor[armL] = PID(PIDarmL, ARM_UP, SensorValue[potL]);
		motor[armR] = PID(PIDarmR, ARM_LOADER, SensorValue[potR]);
		if (PID(PIDarmL, ARM_UP, SensorValue[potL]) > PID_DONE_THRESHOLD || PID(PIDarmL, ARM_UP, SensorValue[potL]) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;

	case ll:
		motor[armL] = PID(PIDarmL, ARM_LOADER, SensorValue[potL]);
		motor[armR] = PID(PIDarmR, ARM_UP, SensorValue[potR]);
		if (PID(PIDarmR, ARM_UP, SensorValue[potR]) > PID_DONE_THRESHOLD || PID(PIDarmR, ARM_UP, SensorValue[potR]) < -PID_DONE_THRESHOLD) armsDone = false;
		else armsDone = true;
		break;
	}
#endif
}

void armsOperatorControl() {
	//Button toggle
	if (vexRT[JOYSTICK_ARM] == 1) {
		if (!armsButtonPressed) {
			armsButtonPressed = true;
			if(currentArmPosition == d) currentArmPosition = u;
			else currentArmPosition = d;
		}
	}
	else armsButtonPressed = false;

	if (vexRT[JOYSTICK_ARM_LOADER] == 1) {
		if (!armsLoaderButtonPressed) {
			armsLoaderButtonPressed = true;
			switch (currentArmPosition) {
			case u:
				currentArmPosition = lr;
				break;
			case d:
				currentArmPosition = ll;
				break;
			case lr:
				currentArmPosition = ll;
				break;
			case ll:
				currentArmPosition = lr;
				break;
			}
		}
	}
	else armsLoaderButtonPressed = false;

	//Based on state of variable 'currentArmPosition', set motors to different values
	armsControl(currentArmPosition);
}

//Claws
void clawsControl(bool state) {
	if (state) {
		clawLoutput = -CLAW_SPEED;
		clawRoutput = CLAW_SPEED;
	}
	else {
		clawLoutput = CLAW_SPEED;
		clawRoutput = -CLAW_SPEED;
	}

	if (clawsCounter <= CLAWS_CYCLES) {
		motor[clawL] = clawLoutput;
		motor[clawR] = clawRoutput;
		clawsDone = false;
	}
	else {
		motor[clawL] = 0;
		motor[clawR] = 0;
		clawsDone = true;
	}

	clawsCounter += (clawsCounter <= CLAWS_CYCLES + 1) ? 1 : 0;
}


void clawsOperatorControl() {
	if (vexRT[JOYSTICK_CLAWS] == 1) {
		if (!clawButtonPressed) {
			clawButtonPressed = true;
			rightClawClosed = !rightClawClosed;
			if (clawsCounter > CLAWS_CYCLES) clawsCounter = 1;
		}
	}
	else clawButtonPressed = false;

	clawsControl(rightClawClosed);
}

//Mobile Goal
void mobileGoalControl(bool state) {
	if (state) {
		mogoLoutput = -127;
		mogoRoutput = 127;
	}
	else {
		mogoLoutput = 127;
		mogoRoutput = -127;
	}

	if (mogoCounter <= MOGO_CYCLES) {
		motor[mogoL] = mogoLoutput;
		motor[mogoR] = mogoRoutput;
		mogoDone = false;
	}
	else if (mogoCounter > MOGO_CYCLES) {
		motor[mogoL] = 0;
		motor[mogoR] = 0;
		mogoDone = true;
	}
	mogoCounter += (mogoCounter <= MOGO_CYCLES + 1) ? 1 : 0;
}

void mobileGoalOperatorControl() {
	if (vexRT[JOYSTICK_MOGO] == 1) {
		if (!mogoButtonPressed) {
			mogoButtonPressed = true;
			mogoRetracted = !mogoRetracted;
			if (mogoCounter > MOGO_CYCLES) mogoCounter = 1;
		}
	}
	else mogoButtonPressed = false;

	mobileGoalControl(mogoRetracted);
}

//LCD
void lcdSelect() {
	if (nLCDButtons == 1) {
		if (!lcdButtonsPressed[2]) {
			lcdButtonsPressed[2] = true;
			switch (currentMenu) {
			case mainMenu:
				currentMenu = batteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case batteryVoltageMenu:
				currentMenu = backupBatteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case backupBatteryVoltageMenu:
				currentMenu = autonomousMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == mogoAndCones)	sprintf(lcdOutput, "< MoGo + Cones >");
				else if (currentCode == cones)	sprintf(lcdOutput, "<     Cones    >");
				else if (currentCode == mogo)	sprintf(lcdOutput, "<  Mobile Goal >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentMenu = allianceColorMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColor == red)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColor == blue)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentMenu = startingSideMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSide == leftSide)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSide == rightSide)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentMenu = mainMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				displayLCDCenteredString(1, "<      OK      >");
				break;
			}
		}
	}
	else lcdButtonsPressed[2] = false;

	if (nLCDButtons == 4) {
		if (!lcdButtonsPressed[0]) {
			lcdButtonsPressed[0] = true;
			switch (currentMenu) {
			case mainMenu:
				currentMenu = startingSideMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSide == leftSide)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSide == rightSide)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentMenu = allianceColorMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColor == red)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColor == blue)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentMenu = autonomousMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == mogoAndCones)	sprintf(lcdOutput, "< MoGo + Cones >");
				else if (currentCode == cones)	sprintf(lcdOutput, "<     Cones    >");
				else if (currentCode == mogo)	sprintf(lcdOutput, "<  Mobile Goal >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentMenu = backupBatteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case backupBatteryVoltageMenu:
				currentMenu = batteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case batteryVoltageMenu:
				currentMenu = mainMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				displayLCDCenteredString(1, "<      OK      >");
				break;
			}
		}
	}
	else lcdButtonsPressed[0] = false;

	if (nLCDButtons == 2) {
		if (!lcdButtonsPressed[1]) {
			lcdButtonsPressed[1] = true;
			switch (currentMenu) {
			case mainMenu:
				lcdReady = true;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				break;

			case batteryVoltageMenu:
				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;
			case backupBatteryVoltageMenu:
				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentCode = (currentCode == cones) ? mogoAndCones :
				(currentCode == mogoAndCones) ? mogo :
				cones;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == mogoAndCones)	sprintf(lcdOutput, "< MoGo + Cones >");
				else if (currentCode == cones)	sprintf(lcdOutput, "<     Cones    >");
				else if (currentCode == mogo)	sprintf(lcdOutput, "<  Mobile Goal >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentColor = currentColor == blue ? red : blue;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColor == red)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColor == blue)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentSide = currentSide == leftSide ? rightSide : leftSide;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSide == leftSide)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSide == rightSide)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			}
		}
	}
	else lcdButtonsPressed[1] = false;
}

#endif
