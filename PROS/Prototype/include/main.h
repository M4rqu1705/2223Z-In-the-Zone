#ifndef MAIN_H_INCLUDED_
#define MAIN_H_INCLUDED_

#include "API.h"
// #include <mogoIntake.h>
// #include <arms.h>
// #include <claws.h>

#include "commons.h"
#include "utils.h"
#include "PID.h"

#include "drive.h"

// #ifdef __cplusplus
// extern "C" {
	// void __libc_init_array();
	// #endif




	void autonomous();
	/**
	 * Runs pre-initialization code. This function will be started in kernel mode one time while the
	 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
	 *
	 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
	 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
	 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
	 */
	void initializeIO();

	void resetValues();
	void initialize();

	void operatorControl();


	// #ifdef __cplusplus
// }
// #endif

#endif
