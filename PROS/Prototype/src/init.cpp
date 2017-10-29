#include "main.hpp"

/*
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO() {
}

void resetValues() {
	//Stop all motors
	motorStopAll();

	//Reset counters
	drive::counter = 0;
	claws::counter = 0;
	mobileGoalIntake::counter = 0;

	//Reset toggle button values to false
	drive::invertButtonPressed = false;
	arms::buttonPressed = false;
	arms::loaderButtonPressed = false;
	claws::buttonPressed = false;
	mobileGoalIntake::buttonPressed = false;

	//Reset drive variables and arrays
	drive::powerOutput = 0;	drive::turnOutput = 0;
	drive::joystickInputs[0] = 0;	drive::joystickInputs[1] = 0;
	drive::outputs[0] = 0;	drive::outputs[1] = 0;
	drive::directionNormal = true;

	//Reset drive sensor values
	encoderReset(drive::encoderL);	encoderReset(drive::encoderR);
	gyroReset(drive::driveGyro);

	//Reset drive PID array values
	pid::PIDdrive[0] = DRIVE_PID_KP_PRESET;	pid::PIDdrive[1] = DRIVE_PID_KI_PRESET; pid::PIDdrive[2] = DRIVE_PID_KD_PRESET;
	pid::PIDdrive[3] = DRIVE_PID_ERROR_PRESET;	pid::PIDdrive[4] = DRIVE_PID_INTEGRAL_PRESET;
	pid::PIDdrive[5] = DRIVE_PID_INTEGRAL_LIMIT_PRESET;	pid::PIDdrive[6] = DRIVE_PID_LAST_ERROR_PRESET;

	//Even if drive previously not done, reset it anyway to make sure the next time it enters the loop, it will run
	drive::notDone = true;

	//Reset arm PID arrays
	pid::PIDarmL[0] = ARM_PID_KP_PRESET;	pid::PIDarmL[1] = ARM_PID_KI_PRESET; pid::PIDarmL[2] = ARM_PID_KD_PRESET;
	pid::PIDarmL[3] = ARM_PID_ERROR_PRESET;	pid::PIDarmL[4] = ARM_PID_INTEGRAL_PRESET;
	pid::PIDarmL[5] = ARM_PID_INTEGRAL_LIMIT_PRESET;	pid::PIDarmL[6] = ARM_PID_LAST_ERROR_PRESET;

	pid::PIDarmR[0] = ARM_PID_KP_PRESET;	pid::PIDarmR[1] = ARM_PID_KI_PRESET; pid::PIDarmR[2] = ARM_PID_KD_PRESET;
	pid::PIDarmR[3] = ARM_PID_ERROR_PRESET;	pid::PIDarmR[4] = ARM_PID_INTEGRAL_PRESET;
	pid::PIDarmR[5] = ARM_PID_INTEGRAL_LIMIT_PRESET;	pid::PIDarmR[6] = ARM_PID_LAST_ERROR_PRESET;

	//Even if arm previously not done, reset it anyway to make sure the next time it enters the loop, it will run
	arms::currentPosition = arms::rightDown;
	arms::notDone = true;

	//Even if claw previously not done, reset it anyway to make sure the next time it enters the loop, it will run
	claws::notDone = true;

	//Even if claws previously not done, reset it anyway to make sure the next time it enters the loop, it will run
	mobileGoalIntake::notDone = true;
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

		drive::encoderL = encoderInit(SENSOR_ENCODER_L, (SENSOR_ENCODER_L + 1), SENSOR_ENCODER_L_INVERTED);
		drive::encoderR = encoderInit(SENSOR_ENCODER_R, (SENSOR_ENCODER_R + 1), SENSOR_ENCODER_R_INVERTED);

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
