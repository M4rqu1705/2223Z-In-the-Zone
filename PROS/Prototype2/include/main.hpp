#ifndef MAIN_HPP_
#define MAIN_HPP_

#include "API.h"
#include "commons.h"

#include "utils.h"

// Allow usage of this file in C++ programs
// #ifdef __cplusplus
// extern "C" {

// #endif

#ifndef DRIVE_H_
#define DRIVE_H_

namespace drive {
	unsigned short counter;
	bool invertButtonPressed;

	//PID drivePID(DRIVE_PID_KP_PRESET, DRIVE_PID_KI_PRESET, DRIVE_PID_KD_PRESET, DRIVE_PID_INTEGRAL_LIMIT_PRESET);

	float slewOutputs[2];  //Declare Slewrate variables to store SLEWCHANGE
	signed short PIDoutput;  //Declare shorts to store the outputs of the PID calculation and the individual drive sides output after rectifying the PID output
	signed char joystickInputs[2], outputs[2];      //Declare array to store joystick values (0 = powerOutput, 1 = turnOutput) so it is not necessary to retrieve the value more than once (efficiency purposes)
	bool directionNormal, notDone;     //Declare booleans to indicate direction state during driver control and if the drive is done moving during autonomous
	enum direction : signed char { forward, backward, turnLeft, turnRight };        //Create enumerated type variable to indicate direciton

	Gyro driveGyro;
	Encoder encoderL;
	Encoder encoderR;

	void reset(bool init = false);

	void operatorControl();
	void move(direction orientation, float pulses, signed char speed, bool useGyro=false);
}

#endif


void autonomous();
/**
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO();
/**
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */
void initialize();
/**
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl();

// End C++ export structure
// #ifdef __cplusplus
// }
// #endif

#endif
