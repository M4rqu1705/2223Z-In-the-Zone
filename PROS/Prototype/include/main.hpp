#ifndef MAIN_HPP_
#define MAIN_HPP_

#include <API.hpp>

#include <constants.hpp>
#include <utils.hpp>
#include <lcd.hpp>
#include <PID.hpp>

#include <drive.hpp>
#include <mogoIntake.hpp>
#include <arms.hpp>
#include <claws.hpp>

#ifdef __cplusplus
extern "C" {
	#endif

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


	#ifdef __cplusplus
}
#endif

#endif
