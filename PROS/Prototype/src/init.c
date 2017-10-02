#include "main.h"

void initializeIO() {
}

void initialize() {
  // Variables
	// Drive
	signed char driveFoutput = 0, driveSoutput = 0, joysDriveFvalue = 0, joysDriveSvalue = 0;
	signed short driveLoutput = 0, driveRoutput = 0;
	bool driveInvertButtonPressed = false, driveDirectionNormal = true;

	gyroInit(SENSOR_GYRO, 0);
	encoderLeft = encoderInit(SENSOR_ENCODER_L, SENSOR_ENCODER_L + 1, SENSOR_ENCODER_L_INVERTED);
	encoderRight = encoderInit(SENSOR_ENCODER_R, SENSOR_ENCODER_R + 1, SENSOR_ENCODER_R_INVERTED);

	// Arm
	bool armButtonPressed = false, rightArmUp = false;
	float filterArmL[] = { 0, 0, 0, 5, 5, 5 };
	float filterArmR[] = { 0, 0, 0, 5, 5, 5 };

	float PIDarmL[] = { 0.75, 0.05, 3.0, getSensor(filterArmL, analogRead(SENSOR_POT_L)), 0, 0, 35, 0 };
	float PIDarmR[] = { 0.75, 0.05, 3.0, getSensor(filterArmR, analogRead(SENSOR_POT_R)), 0, 0, 35, 0 };

	// Claw
	bool clawButtonPressed = false, rightClawClosed = true;
	unsigned char clawCounter = 1;

	//Mobile Goal
	bool mogoButtonPressed = false, mogoRetracted = true;
	unsigned char mogoCounter = 1;
}
