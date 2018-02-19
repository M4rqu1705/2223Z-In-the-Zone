/*   MobileGoalAndDriveTestFunctions2.h - Functions to use with MobileGoalAndDriveTest  *
*    Copyright (C) <2017>  Marcos Ricardo Pesante Colón                                 *
*                                                                                       *
*    This program is free software: you can redistribute it and/or modify               *
*    it under the terms of the GNU General Public License as published by               *
*    the Free Software Foundation, either version 3 of the License, or                  *
*    (at your option) any later version.                                                *
*                                                                                       *
*    This program is distributed in the hope that it will be useful,                    *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                     *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                      *
*    GNU General Public License for more details.                                       *
*                                                                                       *
*    You should have received a copy of the GNU General Public License                  *
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.              */

#ifndef MobileGoalAndDriveTestFunctions2.h
#define MobileGoalAndDriveTestFunctions2.h

#pragma systemfile

//**======================================================**Previously on .h files**======================================================**//

enum direction {Forward = 0, Backward, TurnLeft, TurnRight };
typedef struct{
	bool invertButtonPressed;              //Boolean to store if the joystick drive invert button was pressed in the previous cycle. Used in driver control
	bool directionNormal;                  //Variable to store the current direction with which the robot will move. Used in driver control
	bool notDone;                          //Variable to indicate if the drive is done or not. Used in autonomous
	unsigned byte counter;                 //Variable to count asynchronously to check if drive is done or not. Used in autonomous
	float PID[10];                          //Array to store values to pass to the PID function. Used in autonomous
	signed byte joystickInputs[2];         //Array to store the joystick analgo inputs. 0 is forward input, 1 is side input. Used to not call for the current joystick values various times
	double slewRateOutputs[2];             //Array to store the slew increase or decrease. Used in operator control
	signed byte outputs[2];                //Array to store the future outputs of the motors. 0 is left, 1 is right. Used in both autonomous and user control
}robotDriveStruct;

typedef struct{
	bool retractButtonPressed;  //Boolean to store if the joystick retract button was pressed in the previous cycle. Used in driver control
	bool extendButtonPressed;   //Boolean to store if the joystick extend button was pressed in the previous cycle. Used in driver control
	bool retract;               //Boolean to indicate in which state will the intake of the Mobile Goal be.
	bool notDone;               //Boolean to indicate if the mobile goal intake is done moving or not. Used in operator control
	unsigned byte counter;      //Variable to count asynchronously to check if the intake is done or not. Used in autonomous
	double PID[10];              //Array to store the values to pass to the PID function. Used in autonomous and operator control
	signed byte output;         //Variable to store the future output of the intake of the Mobile Goal. Used in both autonomous and operator control
}robotMobileGoalIntake;

robotDriveStruct drive;
robotMobileGoalIntake mobileGoalIntake;

//Function prototypes
void initialize();
void resetValues();

void driveOperatorControl();
void move(direction orientation, float pulses, signed byte speed);

void moveMobileGoal(bool retract);
void mobileGoalOperatorControl();


bool lcdButtonsPressed[3] = {false, false, false};

//LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD//
#ifdef USING_LCD    //Only include code if USING_LCD is defined in the constants.h file
//Create enumerated type variable lcdMenus to indicate the available menus
enum lcdMenus { mainMenu = 0, batteryVoltageMenu, backupBatteryVoltageMenu, powerExpanderMenu, autonomousMenu, allianceColorMenu, startingSideMenu};
lcdMenus currentMenu = mainMenu;
//Create enumerated type variable autonomousCodes to indicate the things the robot can do (which will then be used to determine which autonomous to execute)
enum autonomousCodes { mogoAndCones = 0, mogo, cones };
autonomousCodes currentCode = mogoAndCones;
//Create enumerated type variable allianceColor to indicate which is the color of the robot's alliance (which will then be used to determine which autonomous to execute)
enum allianceColor { red = 0, blue};
allianceColor currentColor = red;
//Create enumerated type variable startingSide to indicate which side will the autonomous be executed (which will then be used to determine which autonomous to execute)
enum startingSide { leftSide = 0, rightSide};
startingSide currentSide = rightSide;

string lcdOutput;    //Declare variable in which the formatted string containing a variable LCD output will be stored
bool lcdReady;       //Declare a boolean to indicate if autonomous has been selected or not

void lcdSelect();    //Function prototype for the lcdSelect() function which will be declared at the end because of its length
#endif


//**======================================================**Previously on .c files**======================================================**//

#include "utils.h"

//Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset//

//Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize--Initialize//
void initialize(){
	//Only include piece of code if USING_LCD is defined
#ifdef USING_LCD
	clearLCDLine(0);                  //Clear LCD
	clearLCDLine(1);                  //Clear LCD
	bLCDBacklight = LCD_BACKLIGHT;    //Turn backlight on or off based on LCD_BACKLIGHT constant

	currentMenu = mainMenu;        //Preset to main menu
	currentCode = mogoAndCones;    //Preset autonomous to Mobile Goal and Cones
	currentColor = red;            //Preset autonomous alliance color to red
	currentSide = rightSide;       //Preset autonomous starting side to right side

	//None of the LCD buttons have been previously pressed
	lcdButtonsPressed[0] = false;
	lcdButtonsPressed[1] = false;
	lcdButtonsPressed[2] = false;

	if (bIfiRobotDisabled){    //Display that Gyro is calibrating only if robot is disabled
		lcdReady = false;
		displayLCDCenteredString(0, "Calibrating Gyro");
		displayLCDString(1, 0, "...");
	}
	else {                     //If robot is starting up again enabled (maybe it disconnected from the field) don't calibrate gyro and make robot just start again
		lcdReady = true;
	}
	bLCDBacklight = false;
#endif

	//Initialize all motors and sensors using constants from constants.h
	motorType[MOTOR_DRIVE_LB] = MOTOR_DRIVE_LB_TYPE;
	motorType[MOTOR_DRIVE_LF] = MOTOR_DRIVE_LF_TYPE;
	motorType[MOTOR_CLAWS] = MOTOR_CLAWS_TYPE;
	motorType[MOTOR_ARM_LO] = MOTOR_ARM_LO_TYPE;
	motorType[MOTOR_ARM_LI] = MOTOR_ARM_LI_TYPE;
	motorType[MOTOR_ARM_RI] = MOTOR_ARM_RI_TYPE;
	motorType[MOTOR_ARM_RO] = MOTOR_ARM_RO_TYPE;
	motorType[MOTOR_MOBILE_GOAL] = MOTOR_MOBILE_GOAL_TYPE;
	motorType[MOTOR_DRIVE_RF] = MOTOR_DRIVE_RF_TYPE;
	motorType[MOTOR_DRIVE_RB] = MOTOR_DRIVE_RB_TYPE;

	SensorType[SENSOR_ENCODER_L] = sensorQuadEncoder;
	SensorType[SENSOR_ENCODER_R] = sensorQuadEncoder;
	SensorType[POWER_EXPANDER_STATUS] = sensorAnalog;
	SensorType[SENSOR_POT_MOGO] = sensorPotentiometer;
	SensorType[SENSOR_POT_L] = sensorPotentiometer;
	SensorType[SENSOR_POT_R] = sensorPotentiometer;
	resetValues();
	clearDebugStream();

	//Only include piece of code if USING_LCD is defined
#ifdef USING_LCD
	do{
		if(bIfiRobotDisabled)	lcdSelect();
		else break;
	}while (!lcdReady);

	//Clear the LCD
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDCenteredString(0, "     2223-Z     ");    //Output 2223-Z on the screen, signaling that the lcd is done
#endif
}

void resetValues() {
	//Stop all motors
	motor[1] = motor[2] = motor[3] = motor[4] = motor[5] = motor[6] = motor[7] = motor[8] = motor[9] = motor[10] = 0;

	drive.invertButtonPressed = false;
	drive.directionNormal = true;
	drive.notDone = true;

	drive.counter = 0;

	drive.PID[0] = PID_DRIVE_KP_PRESET;
	drive.PID[1] = PID_DRIVE_KI_PRESET;
	drive.PID[2] = PID_DRIVE_KD_PRESET;
	drive.PID[3] = PID_DRIVE_INTEGRAL_MAX_PRESET;
	drive.PID[4] = PID_DRIVE_ERROR_PRESET;
	drive.PID[5] = PID_DRIVE_LAST_ERROR_PRESET;
	drive.PID[6] = PID_DRIVE_INTEGRAL_PRESET;
	drive.PID[7] = PID_DRIVE_CORRECTION_CYCLES;
	drive.PID[8] = PID_DRIVE_DONE_THRESHOLD;
	drive.PID[9] = 0;	//PID output

	drive.joystickInputs[0] = 0;
	drive.joystickInputs[1] = 0;
	drive.slewRateOutputs[0] = 0;
	drive.slewRateOutputs[1] = 0;
	drive.outputs[0] = 0;
	drive.outputs[0] = 0;

	mobileGoalIntake.retractButtonPressed = false;
	mobileGoalIntake.extendButtonPressed = false;
	mobileGoalIntake.notDone = true;
	mobileGoalIntake.retract = true;

	mobileGoalIntake.counter = 0;
	mobileGoalIntake.output = 0;

	mobileGoalIntake.PID[0] = PID_MOBILE_GOAL_EXTEND_KP_PRESET;
	mobileGoalIntake.PID[1] = PID_MOBILE_GOAL_EXTEND_KI_PRESET;
	mobileGoalIntake.PID[2] = PID_MOBILE_GOAL_EXTEND_KD_PRESET;
	mobileGoalIntake.PID[3] = PID_MOBILE_GOAL_EXTEND_INTEGRAL_MAX_PRESET;
	mobileGoalIntake.PID[4] = PID_MOBILE_GOAL_EXTEND_ERROR_PRESET;
	mobileGoalIntake.PID[5] = PID_MOBILE_GOAL_EXTEND_LAST_ERROR_PRESET;
	mobileGoalIntake.PID[6] = PID_MOBILE_GOAL_EXTEND_INTEGRAL_PRESET;
	mobileGoalIntake.PID[7] = PID_MOBILE_GOAL_CORRECTION_CYCLES;
	mobileGoalIntake.PID[8] = PID_MOBILE_GOAL_DONE_THRESHOLD;
	mobileGoalIntake.PID[9] = 0;	//PID output
}

//Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive//
void driveOperatorControl() {

	//Button toggle
	if ((bool)(vexRT[JOYSTICK_DRIVE_INVERT])) {
		if (!drive.invertButtonPressed) {
			drive.invertButtonPressed = true;
			drive.directionNormal = !drive.directionNormal;
		}
	}
	else drive.invertButtonPressed = false;

	//Assign joystick values to variables
	drive.joystickInputs[0] = vexRT[JOYSTICK_DRIVE_F];
	drive.joystickInputs[1] = vexRT[JOYSTICK_DRIVE_S];

	//Only use values if withing threshold. If not withing threshold, assign 0
	if (drive.joystickInputs[0] < DRIVE_THRESHOLD && drive.joystickInputs[0] > -DRIVE_THRESHOLD) drive.joystickInputs[0] = 0;
	if (drive.joystickInputs[1] < DRIVE_THRESHOLD && drive.joystickInputs[1] > -DRIVE_THRESHOLD) drive.joystickInputs[1] = 0;

	//Slewrate control. Assign value to output incrementing or decreasing values using SLEW_GAIN
	if (drive.slewRateOutputs[0] + SLEW_GAIN < drive.joystickInputs[0]) drive.slewRateOutputs[0] += SLEW_GAIN;
	else if (drive.slewRateOutputs[0] - SLEW_GAIN > drive.joystickInputs[0])	drive.slewRateOutputs[0] -= SLEW_GAIN;
	else if (drive.joystickInputs[0] == 0) drive.slewRateOutputs[0] = 0;

	if (drive.slewRateOutputs[1] + SLEW_GAIN < drive.joystickInputs[1]) drive.slewRateOutputs[1] += SLEW_GAIN;
	else if (drive.slewRateOutputs[1] - SLEW_GAIN > drive.joystickInputs[1]) drive.slewRateOutputs[1] -= SLEW_GAIN;
	else if (drive.joystickInputs[1] == 0) drive.slewRateOutputs[1] = 0;

	drive.slewRateOutputs[0] = CLAMP(drive.slewRateOutputs[0]);
	drive.slewRateOutputs[1] = CLAMP(drive.slewRateOutputs[1]);

	if(!drive.directionNormal){
		//Calculate "arcade drive" values for left and right side
		drive.outputs[0] = CLAMP(ROUND(-drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
		drive.outputs[1] = -CLAMP(ROUND(-drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));
	}
	else{
		drive.outputs[0] = CLAMP(ROUND(drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
		drive.outputs[1] = -CLAMP(ROUND(drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));
	}

	//Move motors using calculated values for left and right side
	motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = drive.outputs[0];
	motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = drive.outputs[1];
}

void move(direction orientation, float pulses, signed byte speed) {
	//Recalculate pulses to convert them to degrees of rotation
	if (orientation == TurnLeft || orientation == TurnRight) pulses = DEGREES_ROTATION_TO_ENCODER_PULSES(pulses);
	//Recaluclate pulses to convert them to inches of movement
	else if(orientation == Forward || orientation == Backward) pulses = INCHES_TRANSLATION_TO_ENCODER_PULSES(pulses);

	//Calculate PID and rectify robot if necessary
	calculatePID(drive.PID, pulses, ((abs(SensorValue[SENSOR_ENCODER_L]) + abs(SensorValue[SENSOR_ENCODER_R])) / 2));
	//Rectify with encoders only if moving forward or backwards
	if(orientation == Forward || orientation == Backward) rectifyOutputsEncoder(drive.outputs, drive.PID[9], abs(SensorValue[SENSOR_ENCODER_L]), abs(SensorValue[SENSOR_ENCODER_R]));
	else {
		drive.outputs[0] = drive.PID[9];
		drive.outputs[1] = drive.PID[9];
	}

	//Make sure left and right orientation values are within a range of values between -speed to speed
	drive.outputs[0] = MAP(drive.outputs[0], 127, -127, speed, -speed);
	drive.outputs[1] = MAP(drive.outputs[1], 127, -127, speed, -speed);

	//Move motors based on PID values, direction in which to move
	switch (orientation) {
	case Forward:
		motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = drive.outputs[0];
		motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = -drive.outputs[1];
		break;

	case Backward:
		motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = -drive.outputs[0];
		motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = drive.outputs[1];
		break;

	case TurnLeft:
		motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = -drive.outputs[0];
		motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = -drive.outputs[1];
		break;

	case TurnRight:
		motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = drive.outputs[0];
		motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = drive.outputs[1];
		break;

	}

	//Check if drive is done
	checkIfDone(drive);
}

//Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake//

void mobileGoalOperatorControl(bool simple){
	if(simple){

		if(vexRT[JOYSTICK_MOBILE_GOAL_INTAKE_EXTEND] == 1){
			if(SensorValue[SENSOR_POT_MOGO] < MOBILE_GOAL_EXTENDED_INTAKE) motor[MOTOR_MOBILE_GOAL] = ROUND(MOBILE_GOAL_MAX_OUTPUT*0.9);
			else motor[MOTOR_MOBILE_GOAL] = 0;
		}

		else if(vexRT[JOYSTICK_MOBILE_GOAL_INTAKE_RETRACT] == 1){
			if(SensorValue[SENSOR_POT_MOGO] > MOBILE_GOAL_RETRACTED_INTAKE) motor[MOTOR_MOBILE_GOAL] = -ROUND(MOBILE_GOAL_MAX_OUTPUT*0.9);
			else motor[MOTOR_MOBILE_GOAL] = 0;
		}

		else motor[MOTOR_MOBILE_GOAL] = 0;
	}

	else{
		if(vexRT[JOYSTICK_MOBILE_GOAL_INTAKE_EXTEND] == 1) {
			motor[MOTOR_MOBILE_GOAL] = -127;
			if (!mobileGoalIntake.extendButtonPressed) {
				mobileGoalIntake.extendButtonPressed = true;
				mobileGoalIntake.retract = false;

				mobileGoalIntake.PID[0] = PID_MOBILE_GOAL_EXTEND_KP_PRESET;
				mobileGoalIntake.PID[1] = PID_MOBILE_GOAL_EXTEND_KI_PRESET;
				mobileGoalIntake.PID[2] = PID_MOBILE_GOAL_EXTEND_KD_PRESET;
				mobileGoalIntake.PID[3] = PID_MOBILE_GOAL_EXTEND_INTEGRAL_MAX_PRESET;
				mobileGoalIntake.PID[4] = PID_MOBILE_GOAL_EXTEND_ERROR_PRESET;
				mobileGoalIntake.PID[5] = PID_MOBILE_GOAL_EXTEND_LAST_ERROR_PRESET;
				mobileGoalIntake.PID[6] = PID_MOBILE_GOAL_EXTEND_INTEGRAL_PRESET;
				mobileGoalIntake.PID[7] = PID_MOBILE_GOAL_CORRECTION_CYCLES;
				mobileGoalIntake.PID[8] = PID_MOBILE_GOAL_DONE_THRESHOLD;
				mobileGoalIntake.PID[9] = 0;	//PID output
			}
			calculatePID(mobileGoalIntake.PID, MOBILE_GOAL_EXTENDED_INTAKE, SensorValue[SENSOR_POT_MOGO]);
			motor[MOTOR_MOBILE_GOAL] = mobileGoalIntake.PID[9];
		}
		else if(vexRT[JOYSTICK_MOBILE_GOAL_INTAKE_RETRACT] == 1) {
			motor[MOTOR_MOBILE_GOAL] = 127;
			if (!mobileGoalIntake.retractButtonPressed) {
				mobileGoalIntake.retractButtonPressed = true;
				mobileGoalIntake.retract = true;

				mobileGoalIntake.PID[0] = PID_MOBILE_GOAL_RETRACT_KP_PRESET;
				mobileGoalIntake.PID[1] = PID_MOBILE_GOAL_RETRACT_KI_PRESET;
				mobileGoalIntake.PID[2] = PID_MOBILE_GOAL_RETRACT_KD_PRESET;
				mobileGoalIntake.PID[3] = PID_MOBILE_GOAL_RETRACT_INTEGRAL_MAX_PRESET;
				mobileGoalIntake.PID[4] = PID_MOBILE_GOAL_RETRACT_ERROR_PRESET;
				mobileGoalIntake.PID[5] = PID_MOBILE_GOAL_RETRACT_LAST_ERROR_PRESET;
				mobileGoalIntake.PID[6] = PID_MOBILE_GOAL_RETRACT_INTEGRAL_PRESET;
				mobileGoalIntake.PID[7] = PID_MOBILE_GOAL_CORRECTION_CYCLES;
				mobileGoalIntake.PID[8] = PID_MOBILE_GOAL_DONE_THRESHOLD;
				mobileGoalIntake.PID[9] = 0; //PID output
			}
			calculatePID(mobileGoalIntake.PID, MOBILE_GOAL_RETRACTED_INTAKE, SensorValue[SENSOR_POT_MOGO]);
			motor[MOTOR_MOBILE_GOAL] = mobileGoalIntake.PID[9];
		}
		else {
			mobileGoalIntake.retractButtonPressed = false;
			mobileGoalIntake.extendButtonPressed = false;
			motor[MOTOR_MOBILE_GOAL] = 0;
		}
	}
}

void moveMobileGoal(bool retract){
	if(retract){
		calculatePID(mobileGoalIntake.PID, MOBILE_GOAL_RETRACTED_INTAKE, SensorValue[SENSOR_POT_MOGO]);
	}
	else{
		calculatePID(mobileGoalIntake.PID, MOBILE_GOAL_EXTENDED_INTAKE, SensorValue[SENSOR_POT_MOGO]);
	}

	motor[MOTOR_MOBILE_GOAL] = mobileGoalIntake.PID[9];

	//Check if intake is done
	checkIfDone(mobileGoalIntake);
}

#endif
