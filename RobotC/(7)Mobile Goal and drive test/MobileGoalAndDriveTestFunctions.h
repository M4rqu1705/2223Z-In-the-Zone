#ifndef MobileGoalAndDriveTestFunctions.h
#define MobileGoalAndDriveTestFunctions.h

#pragma systemfile

//**======================================================**Previously on .h files**======================================================**//
enum direction {Forward = 0, Backward, TurnLeft, TurnRight };
typedef struct{
	bool invertButtonPressed;       //Boolean to store if the joystick drive invert button was pressed in the previous cycle. Used in driver control
	bool directionNormal;           //Variable to store the current direction with which the robot will move. Used in driver control
	bool notDone;                   //Variable to indicate if the drive is done or not. Used in autonomous
	unsigned byte counter;          //Variable to count asynchronously to check if drive is done or not. Used in autonomous
	float PID[9];                   //Array to store values to pass to the PID function. Used in autonomous
	signed byte joystickInputs[2];  //Array to store the joystick analgo inputs. 0 is forward input, 1 is side input. Used to not call for the current joystick values various times
	double slewRateOutputs[2];      //Array to store the slew increase or decrease. Used in operator control
	signed byte outputs[2];         //Array to store the future outputs of the motors. 0 is left, 1 is right. Used in both autonomous and user control
}driveStruct;

typedef struct{
	bool retractButtonPressed;  //Boolean to store if the joystick retract button was pressed in the previous cycle. Used in driver control
	bool extendButtonPressed;   //Boolean to store if the joystick extend button was pressed in the previous cycle. Used in driver control
	bool notDone;               //Boolean to indicate if the mobile goal intake is done moving or not. Used in operator control
	unsigned byte counter;      //Variable to count asynchronously to check if the intake is done or not. Used in autonomous
	double PID[9];              //Array to store the values to pass to the PID function. Used in autonomous and operator control
	signed byte output;         //Variable to store the future output of the intake of the Mobile Goal. Used in both autonomous and operator control
	bool retract;               //Boolean to indicate in which state will the intake of the Mobile Goal be.
}robotMobileGoalIntake;

enum possibleArmsPositions {rightUp = 0, leftUp, rightLoader, leftLoader};
typedef struct{
	bool upButtonPressed;
	bool downButtonPressed;
	bool notDone;
	unsigned byte counter;
	float PIDL[9], PIDR[9];
	signed byte outputs[2];
	possibleArmsPositions newPosition;
}robotArms;

driveStruct drive;
robotMobileGoalIntake mobileGoalIntake;
robotArms arms;

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
	drive.PID[8] = 0;	//PID output

	drive.joystickInputs[0] = 0;
	drive.joystickInputs[1] = 0;
	drive.slewRateOutputs[0] = 0;
	drive.slewRateOutputs[1] = 0;
	drive.outputs[0] = 0;
	drive.outputs[0] = 0;


	mobileGoalIntake.retractButtonPressed = false;
	mobileGoalIntake.extendButtonPressed = false;
	mobileGoalIntake.notDone = true;

	mobileGoalIntake.PID[0] = PID_MOBILE_GOAL_KP_PRESET;
	mobileGoalIntake.PID[1] = PID_MOBILE_GOAL_KI_PRESET;
	mobileGoalIntake.PID[2] = PID_MOBILE_GOAL_KD_PRESET;
	mobileGoalIntake.PID[3] = PID_MOBILE_GOAL_INTEGRAL_MAX_PRESET;
	mobileGoalIntake.PID[4] = PID_MOBILE_GOAL_ERROR_PRESET;
	mobileGoalIntake.PID[5] = PID_MOBILE_GOAL_LAST_ERROR_PRESET;
	mobileGoalIntake.PID[6] = PID_MOBILE_GOAL_INTEGRAL_PRESET;
	mobileGoalIntake.PID[7] = PID_MOBILE_GOAL_CORRECTION_CYCLES;
	mobileGoalIntake.PID[8] = 0;	//PID output

	mobileGoalIntake.retract = true;

	arms.upButtonPressed = false;
	arms.downButtonPressed = false;
	arms.notDone = true;

	arms.PIDL[0] = PID_ARMS_KP_PRESET;
	arms.PIDL[1] = PID_ARMS_KI_PRESET;
	arms.PIDL[2] = PID_ARMS_KD_PRESET;
	arms.PIDL[3] = PID_ARMS_INTEGRAL_MAX_PRESET;
	arms.PIDL[4] = PID_ARMS_ERROR_PRESET;
	arms.PIDL[5] = PID_ARMS_LAST_ERROR_PRESET;
	arms.PIDL[6] = PID_ARMS_INTEGRAL_PRESET;
	arms.PIDL[7] = PID_ARMS_CORRECTION_CYCLES;
	arms.PIDL[8] = 0;	//PID output

	arms.PIDR[0] = PID_ARMS_KP_PRESET;
	arms.PIDR[1] = PID_ARMS_KI_PRESET;
	arms.PIDR[2] = PID_ARMS_KD_PRESET;
	arms.PIDR[3] = PID_ARMS_INTEGRAL_MAX_PRESET;
	arms.PIDR[4] = PID_ARMS_ERROR_PRESET;
	arms.PIDR[5] = PID_ARMS_LAST_ERROR_PRESET;
	arms.PIDR[6] = PID_ARMS_INTEGRAL_PRESET;
	arms.PIDR[7] = PID_ARMS_CORRECTION_CYCLES;
	arms.PIDR[8] = 0;	//PID output

	arms.outputs[0] = 0;
	arms.outputs[1] = 0;
}

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

//Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive//
void driveOperatorControl() {

	//Button toggle
	if (vexRT[JOYSTICK_DRIVE_INVERT] == 1) {
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
	CLAMP(drive.slewRateOutputs[0]);

	if (drive.slewRateOutputs[1] + SLEW_GAIN < drive.joystickInputs[1]) drive.slewRateOutputs[1] += SLEW_GAIN;
	else if (drive.slewRateOutputs[1] - SLEW_GAIN > drive.joystickInputs[1]) drive.slewRateOutputs[1] -= SLEW_GAIN;
	else if (drive.joystickInputs[1] == 0) drive.slewRateOutputs[1] = 0;
	CLAMP(drive.slewRateOutputs[1]);

	//Calculate "arcade drive" values for left and right side
	if(drive.directionNormal){
		drive.outputs[0] = CLAMP(ROUND(drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
		drive.outputs[1] = -CLAMP(ROUND(drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));
	}
	else{
		drive.outputs[0] = CLAMP(ROUND(drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));
		drive.outputs[1] = -CLAMP(ROUND(drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
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
	calculatePID(drive, pulses, ((abs(SensorValue[SENSOR_ENCODER_L]) + abs(SensorValue[SENSOR_ENCODER_R])) / 2));
	//Rectify with encoders only if moving forward or backwards
	if(orientation == Forward || orientation == Backward) rectifyOutputsEncoder(drive.outputs, drive.PID[8], abs(SensorValue[SENSOR_ENCODER_L]), abs(SensorValue[SENSOR_ENCODER_R]));
	else {
		drive.outputs[0] = drive.PID[8];
		drive.outputs[1] = drive.PID[8];
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
	if (drive.PID[8] < PID_DONE_THRESHOLD && drive.PID[8] > -PID_DONE_THRESHOLD) {
		if(drive.counter<PID_DRIVE_CORRECTION_CYCLES) drive.counter++;    //Sinchronous counter that doesn't affect other processes
			if(drive.counter==PID_DRIVE_CORRECTION_CYCLES){
			//If DRIVE_PID_CORRECTION_CYCLES time has passed since last time the robot was in position and it is still within the threshold, it means that the drive was done
			calculatePID(drive, pulses, ((abs(SensorValue[SENSOR_ENCODER_L]) + abs(SensorValue[SENSOR_ENCODER_R])) / 2));
			if (drive.PID[8] < PID_DONE_THRESHOLD && drive.PID[8] > -PID_DONE_THRESHOLD)drive.notDone = false;
			else drive.notDone = true;
		}
	}
	else drive.counter = 0;
}

//Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms//
void moveArms(possibleArmsPositions desiredState){
	if(desiredState == rightUp){
		calculatePID(arms, ARM_LEFT_UP, ARM_RIGHT_UP, SensorValue[SENSOR_POT_L], SensorValue[SENSOR_POT_R]);
	}
	else if(desiredState == leftUp){
		calculatePID(arms, ARM_LEFT_DOWN, ARM_RIGHT_DOWN, SensorValue[SENSOR_POT_L], SensorValue[SENSOR_POT_R]);
	}

	motor[MOTOR_ARM_LI] = motor[MOTOR_ARM_LO] = arms.PIDL[8];
	motor[MOTOR_ARM_RI] = motor[MOTOR_ARM_RO] = arms.PIDR[8];

	//Check if arms are done
	if (arms.PIDL[8] < PID_DONE_THRESHOLD && arms.PIDL[8] > -PID_DONE_THRESHOLD && arms.PIDR[8] < PID_DONE_THRESHOLD && arms.PIDR[8] > -PID_DONE_THRESHOLD) {
		if(arms.counter<PID_ARMS_CORRECTION_CYCLES) arms.counter++;    //Sinchronous counter that doesn't affect other processes
			if(arms.counter==PID_ARMS_CORRECTION_CYCLES){
			//If DRIVE_PID_CORRECTION_CYCLES time has passed since last time the robot was in position and it is still within the threshold, it means that the drive was done
			if(desiredState == rightUp){
				calculatePID(arms, ARM_LEFT_UP, ARM_RIGHT_UP, SensorValue[SENSOR_POT_L], SensorValue[SENSOR_POT_R]);
			}
			else if(desiredState == leftUp){
				calculatePID(arms, ARM_LEFT_DOWN, ARM_RIGHT_DOWN, SensorValue[SENSOR_POT_L], SensorValue[SENSOR_POT_R]);
			}
			if(arms.PIDL[8] < PID_DONE_THRESHOLD && arms.PIDL[8] > -PID_DONE_THRESHOLD && arms.PIDR[8] < PID_DONE_THRESHOLD && arms.PIDR[8] > -PID_DONE_THRESHOLD) arms.notDone = false;
			else arms.notDone = true;
		}
	}
	else arms.counter = 0;
}

void armsOperatorControl(){
	if(vexRT[JOYSTICK_ARM_UP] == 1) {
		if (!arms.upButtonPressed) {
			arms.upButtonPressed = true;
			arms.newPosition = rightUp;

			arms.PIDL[0] = PID_ARMS_KP_PRESET;
			arms.PIDL[1] = PID_ARMS_KI_PRESET;
			arms.PIDL[2] = PID_ARMS_KD_PRESET;
			arms.PIDL[3] = PID_ARMS_INTEGRAL_MAX_PRESET;
			arms.PIDL[4] = PID_ARMS_ERROR_PRESET;
			arms.PIDL[5] = PID_ARMS_LAST_ERROR_PRESET;
			arms.PIDL[6] = PID_ARMS_INTEGRAL_PRESET;
			arms.PIDL[7] = PID_ARMS_CORRECTION_CYCLES;
			arms.PIDL[8] = 0;	//PID output
			arms.PIDR[0] = PID_ARMS_KP_PRESET;
			arms.PIDR[1] = PID_ARMS_KI_PRESET;
			arms.PIDR[2] = PID_ARMS_KD_PRESET;
			arms.PIDR[3] = PID_ARMS_INTEGRAL_MAX_PRESET;
			arms.PIDR[4] = PID_ARMS_ERROR_PRESET;
			arms.PIDR[5] = PID_ARMS_LAST_ERROR_PRESET;
			arms.PIDR[6] = PID_ARMS_INTEGRAL_PRESET;
			arms.PIDR[7] = PID_ARMS_CORRECTION_CYCLES;
			arms.PIDR[8] = 0;	//PID output
		}
	}
	else arms.upButtonPressed = false;

	if(vexRT[JOYSTICK_ARM_DOWN] == 1) {
		if (!arms.downButtonPressed) {
			arms.downButtonPressed = true;
			arms.newPosition = leftUp;

			arms.PIDL[0] = PID_ARMS_KP_PRESET;
			arms.PIDL[1] = PID_ARMS_KI_PRESET;
			arms.PIDL[2] = PID_ARMS_KD_PRESET;
			arms.PIDL[3] = PID_ARMS_INTEGRAL_MAX_PRESET;
			arms.PIDL[4] = PID_ARMS_ERROR_PRESET;
			arms.PIDL[5] = PID_ARMS_LAST_ERROR_PRESET;
			arms.PIDL[6] = PID_ARMS_INTEGRAL_PRESET;
			arms.PIDL[7] = PID_ARMS_CORRECTION_CYCLES;
			arms.PIDL[8] = 0;	//PID output
			arms.PIDR[0] = PID_ARMS_KP_PRESET;
			arms.PIDR[1] = PID_ARMS_KI_PRESET;
			arms.PIDR[2] = PID_ARMS_KD_PRESET;
			arms.PIDR[3] = PID_ARMS_INTEGRAL_MAX_PRESET;
			arms.PIDR[4] = PID_ARMS_ERROR_PRESET;
			arms.PIDR[5] = PID_ARMS_LAST_ERROR_PRESET;
			arms.PIDR[6] = PID_ARMS_INTEGRAL_PRESET;
			arms.PIDR[7] = PID_ARMS_CORRECTION_CYCLES;
			arms.PIDR[8] = 0;	//PID output
		}
	}
	else arms.downButtonPressed = false;

	moveArms(arms.newPosition);
}


//Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake//
void moveMobileGoal(bool retract){
	if(retract){
		calculatePID(mobileGoalIntake, MOBILE_GOAL_RETRACTED_INTAKE, SensorValue[SENSOR_POT_MOGO]);
	}
	else{
		calculatePID(mobileGoalIntake, MOBILE_GOAL_EXTENDED_INTAKE, SensorValue[SENSOR_POT_MOGO]);
	}

	motor[MOTOR_MOBILE_GOAL] = mobileGoalIntake.PID[8];

	//Check if intake is done
	if (mobileGoalIntake.PID[8] < PID_DONE_THRESHOLD && mobileGoalIntake.PID[8] > -PID_DONE_THRESHOLD) {
		if(mobileGoalIntake.counter<PID_MOBILE_GOAL_CORRECTION_CYCLES) mobileGoalIntake.counter++;    //Sinchronous counter that doesn't affect other processes
			if(mobileGoalIntake.counter==PID_ARMS_CORRECTION_CYCLES){
			//If DRIVE_PID_CORRECTION_CYCLES time has passed since last time the robot was in position and it is still within the threshold, it means that the drive was done
			if(retract){
				calculatePID(mobileGoalIntake, MOBILE_GOAL_RETRACTED_INTAKE, SensorValue[SENSOR_POT_MOGO]);
			}
			else{
				calculatePID(mobileGoalIntake, MOBILE_GOAL_EXTENDED_INTAKE, SensorValue[SENSOR_POT_MOGO]);
			}
			if(mobileGoalIntake.PID[8] < PID_DONE_THRESHOLD && mobileGoalIntake.PID[8] > -PID_DONE_THRESHOLD) mobileGoalIntake.notDone = false;
			else mobileGoalIntake.notDone = true;
		}
	}
	else mobileGoalIntake.counter = 0;
}


void mobileGoalOperatorControl(){
	if(vexRT[JOYSTICK_MOBILE_GOAL_INTAKE_EXTEND] == 1) {
		motor[MOTOR_MOBILE_GOAL] = -127;
		/*if (!mobileGoalIntake.extendButtonPressed) {
			mobileGoalIntake.extendButtonPressed = true;
			mobileGoalIntake.retract = false;

			mobileGoalIntake.PID[0] = PID_MOBILE_GOAL_KP_PRESET;
			mobileGoalIntake.PID[1] = PID_MOBILE_GOAL_KI_PRESET;
			mobileGoalIntake.PID[2] = PID_MOBILE_GOAL_KD_PRESET;
			mobileGoalIntake.PID[3] = PID_MOBILE_GOAL_INTEGRAL_MAX_PRESET;
			mobileGoalIntake.PID[4] = PID_MOBILE_GOAL_ERROR_PRESET;
			mobileGoalIntake.PID[5] = PID_MOBILE_GOAL_LAST_ERROR_PRESET;
			mobileGoalIntake.PID[6] = PID_MOBILE_GOAL_INTEGRAL_PRESET;
			mobileGoalIntake.PID[7] = PID_MOBILE_GOAL_CORRECTION_CYCLES;
			mobileGoalIntake.PID[8] = 0;	//PID output
		}*/
	}

	else if(vexRT[JOYSTICK_MOBILE_GOAL_INTAKE_RETRACT] == 1) {
		motor[MOTOR_MOBILE_GOAL] = 127;
		/*if (!mobileGoalIntake.retractButtonPressed) {
			mobileGoalIntake.retractButtonPressed = true;
			mobileGoalIntake.retract = true;

			mobileGoalIntake.PID[0] = PID_MOBILE_GOAL_KP_PRESET;
			mobileGoalIntake.PID[1] = PID_MOBILE_GOAL_KI_PRESET;
			mobileGoalIntake.PID[2] = PID_MOBILE_GOAL_KD_PRESET;
			mobileGoalIntake.PID[3] = PID_MOBILE_GOAL_INTEGRAL_MAX_PRESET;
			mobileGoalIntake.PID[4] = PID_MOBILE_GOAL_ERROR_PRESET;
			mobileGoalIntake.PID[5] = PID_MOBILE_GOAL_LAST_ERROR_PRESET;
			mobileGoalIntake.PID[6] = PID_MOBILE_GOAL_INTEGRAL_PRESET;
			mobileGoalIntake.PID[7] = PID_MOBILE_GOAL_CORRECTION_CYCLES;
			mobileGoalIntake.PID[8] = 0; //PID output
		}*/
	}
	else {
		//mobileGoalIntake.retractButtonPressed = false;
		motor[MOTOR_MOBILE_GOAL] = 0;
	}

	//moveMobileGoal(mobileGoalIntake.retract);
}

#endif
