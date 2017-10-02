#ifndef MAIN_H_
#define MAIN_H_

#include <API.h>
#include <cmath>

#include "PID.h"
#include "kalmanFilter.h"

#ifdef _cplusplus
extern "C" {
	#endif

	//-Constants---------------------------------------------------------------------------------------------------------------------//
	#include "constants.h"

	//-Variables---------------------------------------------------------------------------------------------------------------------//
	// Drive
	signed char driveFoutput, driveSoutput, joysDriveFvalue, joysDriveSvalue;
	signed short driveLoutput, driveRoutput;
	bool driveInvertButtonPressed, driveDirectionNormal;

	Encoder encoderLeft; Encoder encoderRight;
	Gyro driveGyro;

	// Arm
	bool armButtonPressed, rightArmUp;

	//Claw
	bool clawButtonPressed, rightClawClosed;
	unsigned char clawCounter;

	//Mobile Goal
	bool mogoButtonPressed, mogoRetracted;
	unsigned char mogoCounter;


	//-Functions---------------------------------------------------------------------------------------------------------------------//
	// Claw
	void clawControl(bool state);
	//Mobile Goal
	void mobileGoalControl(bool state);

	//Program functions
	void autonomous();

	/*pinMode()) and port states (digitalWrite()) of limit switches, push buttons,
	* and solenoids. It can also safely
	* configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()). */
	void initializeIO();

	/* This function should initialize most sensors (gyro, encoders, ultrasonics),
	 * LCDs, global variables, and IMEs like the pre_auton() in other environments
	 */
	void initialize();

	void operatorControl();

	// End C++ export structure
	#ifdef _cplusplus
}
#endif
#endif
