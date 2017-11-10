#include "main.h"

/*
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO() {
	__libc_init_array();
}

void resetValues() {
	//Stop all motors
	motorStopAll();

	drive::init(false);
	// arms::init();
	// claws::init();
	// mobileGoalIntake::init();
}

void initialize() {
	//Only include piece of code if USING_LCD is defined
	#if USING_LCD
	lcdInit(LCD_PORT);                //Initialize LCD
	lcdClear(LCD_PORT);               //Clear LCD
	lcdSetBacklight(LCD_PORT, LCD_BACKLIGHT);    //Turn backlight on or off based on LCD_BACKLIGHT constant

	LCD::currentMenu = mainMenu;        //Preset to main menu
	LCD::currentCode = mogoAndCones;    //Preset autonomous to Mobile Goal and Cones
	LCD::currentColor = red;            //Preset autonomous alliance color to red
	LCD::currentSide = rightSide;       //Preset autonomous starting side to right side
	//None of the LCD buttons have been previously pressed
	LCD::lcdButtonsPressed[0] = false;
	LCD::lcdButtonsPressed[1] = false;
	LCD::lcdButtonsPressed[2] = false;

	if (!isEnabled()) {    //Display that Gyro is calibrating only if robot is disabled
		LCD::notDone = true;
		lcdSetText(LCD_PORT, 0, "Calibrating Gyro")
			lcdSetText(LCD_PORT, 1, "      ...       ");
	}
	else {                     //If robot is starting up again enabled (maybe it disconnected from the field) don't calibrate gyro and make robot just start again
		LCD::notDone = false;
	}
	lcdSetBacklight(LCD_PORT, false)
		#endif

	if (!isEnabled()) {                    //Only calibrate Gyro if robot is disabled
		drive::driveGyro = gyroInit(SENSOR_GYRO, GYRO_FULL_ROTATION_TICKS);
	}

	resetValues();

	//Only include piece of code if USING_LCD is defined
	#if USING_LCD
	do {
		if (!isEnabled())	LCD::select();
		else break;
	} while (LCD::notDone);

	//Clear the LCD
	lcdClear(LCD_PORT);
	lcdSetText(LCD_PORT, 0, "     2223-Z     ");    //Output 2223-Z on the screen, signaling that the lcd is done
	lcdShutdown(LCD_PORT);
	#endif

}
