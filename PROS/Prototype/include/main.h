#ifndef MAIN_H_
#define MAIN_H_

#include <API.h>
#include <cmath>

#include "PID.h"
#include "kalmanFilter.h"
#include "lcd.h"
#include "conversions.h"
#include "drive.h"
#include "arms.h"
#include "claws.h"
#include "mobileGoal.h"

#ifdef _cplusplus
extern "C" {
	#endif

	//-Constants---------------------------------------------------------------------------------------------------------------------//
	#include "constants.h"

	//-Functions---------------------------------------------------------------------------------------------------------------------//

	void resetValues();
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
