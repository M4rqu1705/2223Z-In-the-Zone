#ifndef functions.h
#define functions.h

#pragma systemfile

//**======================================================**Previously on .h files**======================================================**//
//Counters
typedef unsigned byte counter;                      //Create datatype designed for counters called counter
counter driveCounter, clawsCounter, mogoCounter;    //Declare counters

//Toggle button
typedef bool toggleButton;    //Create datatype designed for button toggles
//Declare toggle button variables
toggleButton driveInvertButtonPressed, armsButtonPressed, armsLoaderButtonPressed, clawsButtonPressed, mogoButtonPressed;
toggleButton lcdButtonsPressed[3] = { false, false, false };

//PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID//
float PIDdrive[7], PIDarmL[7], PIDarmR[7];    //Declare PID arrays for drive and arms
signed short output;

//Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive//
float drivePowerOutput, driveTurnOutput;  //Declare Slewrate variables to store SLEWCHANGE
signed short PIDoutput, driveOutputs[2];  //Declare shorts to store the outputs of the PID calculation and the individual drive sides output after rectifying the PID output
signed  byte joystickDriveInputs[2];      //Declare array to store joystick values (0 = powerOutput, 1 = turnOutput) so it is not necessary to retrieve the value more than once (efficiency purposes)
bool driveDirectionNormal, driveDone;     //Declare booleans to indicate direction state during driver control and if the drive is done moving during autonomous
enum direction { f = 0, b, l, r };        //Create enumerated type variable to indicate direciton

//Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms//
bool armsDone;                                          //Declare booleans to indicate if the arm is done moving during autonomous
enum armsPositions { d = 0, u = 1, lr = 2, ll = 3 };    //Declare enumerated type variable to indicate arm positions
armsPositions currentArmPosition = d;

//Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws//
bool rightClawClosed, clawsDone;    //Declare booleans to indicate claw positions and if the claws are done moving during autonomous
signed short clawLoutput, clawRoutput;     //Declare shorts to store the speed in which to move each on of the claws

//Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake//
bool mogoRetracted, mogoDone;      //Declare booleans to indicate if mobile goal is currently retracted or not (if, when activated, it will respectively extend or retract) and to indicate if the mobile goal intake is done moving during autonomous
signed short mogoLoutput, mogoRoutput;    //Declare shorts to store the speed in which to move each side of the Mobile Goal intake

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
//Conversions
signed int ROUND(float inNumber){
	if(inNumber>=0) return ceil(inNumber - 0.49);	//Retreived from http://www.cplusplus.com/forum/beginner/3600/
	else return ceil(inNumber+0.49);
}

signed short MAP(signed short inNumber, signed short inMin, signed short inMax, signed short outMin, signed short outMax) {
	return ((inNumber - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}
//Retreived from https://www.arduino.cc/en/Reference/Map

signed byte CLAMP(signed short inNumber) {
	if(inNumber > 127) return 127;
	else if (inNumber < -127) return -127;
	return inNumber;
}

unsigned short INCHES_TRANSLATION_TO_ENCODER_PULSES(float inches) {
	return ROUND(inches*(360/(WHEEL_DIAMETER*PI)));
}

signed short DEGREES_ROTATION_TO_GYRO_TICKS(float targetDegrees) {
	return ROUND(targetDegrees *(GYRO_FULL_ROTATION_TICKS / 360));
}

signed short DEGREES_ROTATION_TO_ENCODER_PULSES(float targetDegrees) {
	return ROUND((targetDegrees*DRIVE_WIDTH)/WHEEL_DIAMETER);
}
//Retreived from https://en.wikipedia.org/wiki/Arc_(geometry)#Length_of_an_arc_of_a_circle

void rectifyOutputsEncoder(signed short *values, signed byte speed, signed short leftSideSensor, signed short rightSideSensor) {
	values[0] = speed - (leftSideSensor - rightSideSensor)*RECTIFY_CONSTANT_ENCODER;
	values[1] = speed - (rightSideSensor - leftSideSensor)*RECTIFY_CONSTANT_ENCODER;
}

void rectifyDriveGyro(short *values, signed byte speed, signed short gyroSensor){
	values[0] = speed - ((gyroSensor)*RECTIFY_CONSTANT_GYRO);
	values[1] = speed + ((gyroSensor)*RECTIFY_CONSTANT_GYRO);
}

//Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset--Reset//
void resetValues() {
	//Stop all motors
	motor[1] = motor[2] = motor[3] = motor[4] = motor[5] = motor[6] = motor[7] = motor[8] = motor[9] = motor[10] = 0;

	//Reset counters
	driveCounter = 0;
	clawsCounter = 0;
	mogoCounter = 0;

	//Reset toggle button values to false
	driveInvertButtonPressed = false;
	armsButtonPressed = false;
	armsLoaderButtonPressed = false;
	clawsButtonPressed = false;
	mogoButtonPressed = false;

	//Reset drive variables and arrays
	drivePowerOutput = 0;	driveTurnOutput = 0;
	joystickDriveInputs[0] = 0;	joystickDriveInputs[1] = 0;
	driveOutputs[0] = 0;	driveOutputs[1] = 0;
	driveDirectionNormal = true;

	//Reset drive sensor values
	SensorValue[SENSOR_ENCODER_L] = 0;	SensorValue[SENSOR_ENCODER_R]  = 0;
	SensorValue[SENSOR_GYRO] = 0 ;

	//Reset drive PID array values
	PIDdrive[0] = DRIVE_PID_KP_PRESET;	PIDdrive[1] = DRIVE_PID_KI_PRESET; PIDdrive[2] = DRIVE_PID_KD_PRESET;
	PIDdrive[3] = DRIVE_PID_ERROR_PRESET;	PIDdrive[4] = DRIVE_PID_INTEGRAL_PRESET;
	PIDdrive[5] = DRIVE_PID_INTEGRAL_LIMIT_PRESET;	PIDdrive[6] = DRIVE_PID_LAST_ERROR_PRESET;

	//Even if drive previously not done, reset it anyway to make sure the next time it enters the loop, it will run
	driveDone = false;

	//Reset arm PID arrays
	PIDarmL[0] = ARM_PID_KP_PRESET;	PIDarmL[1] = ARM_PID_KI_PRESET; PIDarmL[2] = ARM_PID_KD_PRESET;
	PIDarmL[3] = ARM_PID_ERROR_PRESET;	PIDarmL[4] = ARM_PID_INTEGRAL_PRESET;
	PIDarmL[5] = ARM_PID_INTEGRAL_LIMIT_PRESET;	PIDarmL[6] = ARM_PID_LAST_ERROR_PRESET;

	PIDarmR[0] = ARM_PID_KP_PRESET;	PIDarmR[1] = ARM_PID_KI_PRESET; PIDarmR[2] = ARM_PID_KD_PRESET;
	PIDarmR[3] = ARM_PID_ERROR_PRESET;	PIDarmR[4] = ARM_PID_INTEGRAL_PRESET;
	PIDarmR[5] = ARM_PID_INTEGRAL_LIMIT_PRESET;	PIDarmR[6] = ARM_PID_LAST_ERROR_PRESET;

	//Even if arm previously not done, reset it anyway to make sure the next time it enters the loop, it will run
	armsDone = false;

	//Even if claw previously not done, reset it anyway to make sure the next time it enters the loop, it will run
	clawsDone = false;

	//Even if claws previously not done, reset it anyway to make sure the next time it enters the loop, it will run
	mogoDone = false;
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
	motorType[MOTOR_MOGO_L] = MOTOR_MOGO_L_TYPE;
	motorType[MOTOR_DRIVE_LF] = MOTOR_DRIVE_LF_TYPE;
	motorType[MOTOR_DRIVE_LB] = MOTOR_DRIVE_LB_TYPE;
	motorType[MOTOR_ARM_L] = MOTOR_ARM_L_TYPE;
	motorType[MOTOR_CLAW_L] = MOTOR_CLAW_L_TYPE;
	motorType[MOTOR_CLAW_R] = MOTOR_CLAW_R_TYPE;
	motorType[MOTOR_ARM_R] = MOTOR_ARM_R_TYPE;
	motorType[MOTOR_DRIVE_RB] = MOTOR_DRIVE_RB_TYPE;
	motorType[MOTOR_DRIVE_RF] = MOTOR_DRIVE_RF_TYPE;
	motorType[MOTOR_MOGO_R] = MOTOR_MOGO_R_TYPE;

	SensorType[SENSOR_ENCODER_L] = sensorQuadEncoder;
	SensorType[SENSOR_ENCODER_R] = sensorQuadEncoder;
	SensorType[POWER_EXPANDER_STATUS] = sensorAnalog;
	SensorType[SENSOR_POT_L] = sensorPotentiometer;
	SensorType[SENSOR_POT_R] = sensorPotentiometer;

	if(bIfiRobotDisabled){                    //Only calibrate Gyro if robot is disabled
		SensorType[SENSOR_GYRO] = sensorNone;   //Reset SENSOR_GYRO port
		wait1Msec(500);
		SensorType[SENSOR_GYRO] = sensorGyro;   //Assign gyro sensor to SENSOR_GYRO port
		wait1Msec(1500);                        //Wait for it to calibrate

		//SensorScale[in8] = 260;		//Adjust SensorScale to correct the scaling for your SENSOR_GYRO
		SensorFullCount[in8] = GYRO_FULL_ROTATION_TICKS;	//Adjust SensorFullCount to set the "rollover" point. A value of 3600 sets the rollover point to +/-3600
	}
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

//PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID--PID//
byte PID(float *values, signed short target, signed int sensorInput) {
	//values[] array format: KP(0),  KI(1), KD(2), error(3), integral(4), integralLimit(5), lastError(6)

	//Calculate error
	values[3] = (float)(target - sensorInput);

	//Calculate integral if within the integralLimit range
	values[4] += (abs(values[4]) + abs(values[3]) < values[5]) ? values[3] : 0;
	//In the case that error is 0, reset the integral
	if (values[3] == 0) values[4] = 0;

	//Calculate the output adding KP by error, KI by integral and KD by derivative
	//Just so the Integral doesn't grow too fast, map an exagerated number to fit values from -127 to 127
	//Calculate the derivative here so there is no need to store it in a variable
	output = values[0] * values[3] + MAP((values[1] * values[4]), -(values[5]), values[5], -127, 127) + values[2] * (values[3]-values[6]);

	values[6] = values[3];

	return CLAMP(output);
}

//Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive--Drive//
void driveOperatorControl() {

	//Button toggle
	if (vexRT[JOYSTICK_DRIVE_INVERT] == 1) {
		if (!driveInvertButtonPressed) {
			driveInvertButtonPressed = true;
			driveDirectionNormal = !driveDirectionNormal;
		}
	}
	else driveInvertButtonPressed = false;

	//Assign joystick values to variables
	joystickDriveInputs[0] = vexRT[JOYSTICK_DRIVE_F];
	joystickDriveInputs[1] = vexRT[JOYSTICK_DRIVE_S];

	//Only use values if withing threshold. If not withing threshold, assign 0
	if (joystickDriveInputs[0] < DRIVE_THRESHOLD && joystickDriveInputs[0] > -DRIVE_THRESHOLD) joystickDriveInputs[0] = 0;
	if (joystickDriveInputs[1] < DRIVE_THRESHOLD && joystickDriveInputs[1] > -DRIVE_THRESHOLD) joystickDriveInputs[1] = 0;

	//Slewrate control. Assign value to output incrementing or decreasing values using SLEW_GAIN
	//If direction inverted, decrease values where normally increased and vice versa
	if (drivePowerOutput + SLEW_GAIN < joystickDriveInputs[0])drivePowerOutput += SLEW_GAIN;
	else if (drivePowerOutput - SLEW_GAIN > joystickDriveInputs[0])	drivePowerOutput -= SLEW_GAIN;

	if (driveTurnOutput + SLEW_GAIN < joystickDriveInputs[1]) driveTurnOutput += SLEW_GAIN;
	else if (driveTurnOutput - SLEW_GAIN > joystickDriveInputs[1]) driveTurnOutput -= SLEW_GAIN;

	//Calculate "arcade drive" values for left and right side
	if(driveDirectionNormal){
		driveOutputs[0] = ROUND(driveTurnOutput) + ROUND(drivePowerOutput);
		driveOutputs[1] = ROUND(driveTurnOutput) - ROUND(drivePowerOutput);
	}
	else{
		driveOutputs[0] = ROUND(driveTurnOutput) - ROUND(drivePowerOutput);
		driveOutputs[1] = ROUND(driveTurnOutput) + ROUND(drivePowerOutput);
	}

	//Make sure left and right side values are within a range of values between -127 to 127
	driveOutputs[0] = CLAMP(driveOutputs[0]);
	driveOutputs[1] = CLAMP(driveOutputs[1]);

	//Move motors using calculated values for left and right side
	motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = driveOutputs[0];
	motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = driveOutputs[1];
}

void drive(direction orientation, float pulses, signed byte speed, bool useGyro = false) {
	//Recalculate pulses to convert them to degrees of rotation
	if (orientation == l || orientation == r) {
		if (useGyro) pulses = DEGREES_ROTATION_TO_GYRO_TICKS(pulses);
		else pulses = DEGREES_ROTATION_TO_ENCODER_PULSES(pulses);
	}
	//Recaluclate pulses to convert them to inches of movement
	else if(orientation == f || orientation == b) pulses = INCHES_TRANSLATION_TO_ENCODER_PULSES(pulses);

	//Calculate PID and rectify robot if necessary
	if(useGyro){
		if(orientation == f || orientation == b){    //Calculate PID using encoder values and rectify with gyro
			PIDoutput = PID(PIDdrive, pulses, ((abs(SensorValue[SENSOR_ENCODER_L]) + abs(SensorValue[SENSOR_ENCODER_R])) / 2));
			rectifyDriveGyro(driveOutputs, PIDoutput, SensorValue[SENSOR_GYRO]);
		}
		else if (orientation == l || orientation == r){    //Calculate PID using gyro values and don't rectify
			PIDoutput = PID(PIDdrive, pulses, SensorValue[SENSOR_GYRO]);
			driveOutputs[0] = PIDoutput;
			driveOutputs[1] = PIDoutput;
		}
	}
	else{
		//Calculate PID using encoder values
		PIDoutput = PID(PIDdrive, pulses, ((abs(SensorValue[SENSOR_ENCODER_L]) + abs(SensorValue[SENSOR_ENCODER_R])) / 2));
		//Rectify with encoders only if moving forward or backwards
		if(orientation == f || orientation == b) rectifyOutputsEncoder(driveOutputs, PIDoutput, abs(SensorValue[SENSOR_ENCODER_L]), abs(SensorValue[SENSOR_ENCODER_R]));
		else {
			driveOutputs[0] = PIDoutput;
			driveOutputs[1] = PIDoutput;
		}
	}

	//PIDoutput = PID(PIDdrive, pulses, ((abs(SensorValue[SENSOR_ENCODER_L]) + abs(SensorValue[SENSOR_ENCODER_R])) / 2));
	//driveOutputs[0] = PIDoutput;
	//driveOutputs[1] = PIDoutput;

	//Make sure left and right orientation values are within a range of values between -speed to speed
	driveOutputs[0] = MAP(CLAMP(driveOutputs[0]), -127, 127, -speed, speed);
	driveOutputs[1] = MAP(CLAMP(driveOutputs[1]), -127, 127, -speed, speed);

	//Move motors based on PID values, direction in which to move
	switch (orientation) {
	case f:
		motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = driveOutputs[0];
		motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = -driveOutputs[1];
		break;

	case b:
		motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = -driveOutputs[0];
		motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = driveOutputs[1];
		break;

	case l:
		motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = -driveOutputs[0];
		motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = -driveOutputs[1];
		break;

	case r:
		motor[MOTOR_DRIVE_LF] = motor[MOTOR_DRIVE_LB] = driveOutputs[0];
		motor[MOTOR_DRIVE_RF] = motor[MOTOR_DRIVE_RB] = driveOutputs[1];
		break;

	}

	//Check if drive is done
	if (PIDoutput < PID_DONE_THRESHOLD && PIDoutput > -PID_DONE_THRESHOLD) {
		if(driveCounter<DRIVE_PID_CORRECTION_CYCLES) driveCounter++;    //Sinchronous counter that doesn't affect other processes
			if(driveCounter==DRIVE_PID_CORRECTION_CYCLES){
			//If DRIVE_PID_CORRECTION_CYCLES time has passed since last time the robot was in position and it is still withing the threshold, it means that the drive was done
			PIDoutput = PID(PIDdrive, pulses, ((abs(SensorValue[SENSOR_ENCODER_L]) + abs(SensorValue[SENSOR_ENCODER_R])) / 2));
			if (PIDoutput < PID_DONE_THRESHOLD && PIDoutput > -PID_DONE_THRESHOLD){
				driveDone = true;
				if(useGyro)SensorValue[SENSOR_GYRO] = 0;
			}
		}
		else driveDone = false;
	}
	else driveCounter = 0;
}

//Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms--Arms//
void armsControl(armsPositions state) {
	//Based on state of variable 'state', set motors to different values

	switch (state) {
	case u:
		//Move motors based on PID calculated values
		motor[MOTOR_ARM_L] = CLAMP(PID(PIDarmL, ARM_DOWN, SensorValue[SENSOR_POT_L]));
		motor[MOTOR_ARM_R] = CLAMP(PID(PIDarmR, ARM_UP, SensorValue[SENSOR_POT_R]));

		//Check if arm is in position within the threshold
		if (PID(PIDarmR, ARM_UP, SensorValue[SENSOR_POT_R]) < PID_DONE_THRESHOLD && PID(PIDarmR, ARM_UP, SensorValue[SENSOR_POT_R]) > -PID_DONE_THRESHOLD) armsDone = true;
		else armsDone = false;
		break;

	case d:
		//Move motors based on PID calculated values
		motor[MOTOR_ARM_L] = CLAMP(PID(PIDarmL, ARM_UP, SensorValue[SENSOR_POT_L]));
		motor[MOTOR_ARM_R] = CLAMP(PID(PIDarmR, ARM_DOWN, SensorValue[SENSOR_POT_R]));

		//Check if arm is in position within the threshold
		if (PID(PIDarmL, ARM_UP, SensorValue[SENSOR_POT_L]) < PID_DONE_THRESHOLD && PID(PIDarmL, ARM_UP, SensorValue[SENSOR_POT_L]) > -PID_DONE_THRESHOLD) armsDone = true;
		else armsDone = false;
		break;

	case lr:
		//Move motors based on PID calculated values
		motor[MOTOR_ARM_L] = CLAMP(PID(PIDarmL, ARM_UP, SensorValue[SENSOR_POT_L]));
		motor[MOTOR_ARM_R] = CLAMP(PID(PIDarmR, ARM_LOADER, SensorValue[SENSOR_POT_R]));

		//Check if arm is in position within the threshold
		if (PID(PIDarmL, ARM_UP, SensorValue[SENSOR_POT_L]) < PID_DONE_THRESHOLD && PID(PIDarmL, ARM_UP, SensorValue[SENSOR_POT_L]) > -PID_DONE_THRESHOLD) armsDone = true;
		else armsDone = false;
		break;

	case ll:
		//Move motors based on PID calculated values
		motor[MOTOR_ARM_L] = CLAMP(PID(PIDarmL, ARM_LOADER, SensorValue[SENSOR_POT_L]));
		motor[MOTOR_ARM_R] = CLAMP(PID(PIDarmR, ARM_UP, SensorValue[SENSOR_POT_R]));

		//Check if arm is in position within the threshold
		if (PID(PIDarmR, ARM_UP, SensorValue[SENSOR_POT_R]) < PID_DONE_THRESHOLD && PID(PIDarmR, ARM_UP, SensorValue[SENSOR_POT_R]) > -PID_DONE_THRESHOLD) armsDone = true;
		else armsDone = false;
		break;
	}
}

void armsOperatorControl() {
	//Button toggle for "normal button"
	if (vexRT[JOYSTICK_ARM] == 1) {
		if (!armsButtonPressed) {
			armsButtonPressed = true;
			if(currentArmPosition == d) currentArmPosition = u;
			else currentArmPosition = d;
		}
	}
	else armsButtonPressed = false;

	//Button toggle for "loader button"
	if (vexRT[JOYSTICK_ARM_LOADER] == 1) {
		if (!armsLoaderButtonPressed) {
			armsLoaderButtonPressed = true;
			switch (currentArmPosition) {
			case u:
				currentArmPosition = lr;
				break;
			case d:
				currentArmPosition = ll;
				break;
			case lr:
				currentArmPosition = ll;
				break;
			case ll:
				currentArmPosition = lr;
				break;
			}
		}
	}
	else armsLoaderButtonPressed = false;

	//Based on state of variable 'currentArmPosition', set motors to different values
	armsControl(currentArmPosition);
}

//Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws--Claws//
void clawsControl(bool state) {
	//Reevaluate the claw's output per side based on the current state of 'state'
	if (state) {
		clawLoutput = -CLAW_SPEED;
		clawRoutput = CLAW_SPEED;
	}
	else {
		clawLoutput = CLAW_SPEED;
		clawRoutput = -CLAW_SPEED;
	}

	//If CLAWS_CYCLES time has not gone by, keep moving claws
	if (clawsCounter < CLAWS_CYCLES) {
		motor[MOTOR_CLAW_L] = clawLoutput;
		motor[MOTOR_CLAW_R] = clawRoutput;
		clawsCounter+=1;
		clawsDone = false;
	}
	//If CLAWS_CYLES time has not gone by, stop moving claws and make clawsDone true
	else {
		motor[MOTOR_CLAW_L] = 0;
		motor[MOTOR_CLAW_R] = 0;
		clawsDone = true;
	}
}

void clawsOperatorControl() {
	//Toggle button to change the claw's position
	if (vexRT[JOYSTICK_CLAWS] == 1) {
		if (!clawsButtonPressed) {
			clawsButtonPressed = true;
			rightClawClosed = !rightClawClosed;
			if (clawsCounter == CLAWS_CYCLES) clawsCounter = 0;
		}
	}
	else clawsButtonPressed = false;

	//Call clawsControl function with updated state of claws
	clawsControl(rightClawClosed);
}

//Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake -- Mobile Goal Intake//
void mobileGoalControl(bool state) {
	if (state) {
		mogoLoutput = -127;
		mogoRoutput = 127;
	}
	else {
		mogoLoutput = 127;
		mogoRoutput = -127;
	}

	if (mogoCounter < MOGO_CYCLES) {
		motor[MOTOR_MOGO_L] = mogoLoutput;
		motor[MOTOR_MOGO_R] = mogoRoutput;
		mogoCounter+=1;
		mogoDone = false;
	}
	else if (mogoCounter == MOGO_CYCLES) {
		motor[MOTOR_MOGO_L] = 0;
		motor[MOTOR_MOGO_R] = 0;
		mogoDone = true;
	}
}

void mobileGoalOperatorControl() {
	if (vexRT[JOYSTICK_MOGO] == 1) {
		if (!mogoButtonPressed) {
			mogoButtonPressed = true;
			mogoRetracted = !mogoRetracted;
			if (mogoCounter == MOGO_CYCLES) mogoCounter = 0;
		}
	}
	else mogoButtonPressed = false;

	mobileGoalControl(mogoRetracted);
}

//LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD--LCD//
#ifdef USING_LCD
void lcdSelect() {
	if (nLCDButtons == 4) {
		if (!lcdButtonsPressed[2]) {
			lcdButtonsPressed[2] = true;
			switch (currentMenu) {
			case mainMenu:
				currentMenu = batteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case batteryVoltageMenu:
				currentMenu = backupBatteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case backupBatteryVoltageMenu:
				currentMenu = powerExpanderMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Power Expander V");
				sprintf(lcdOutput, "<     %1.2f%s", SensorValue[POWER_EXPANDER_STATUS]/POWER_EXPANDER_DIVISOR,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case powerExpanderMenu:
				currentMenu = autonomousMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == mogoAndCones)	sprintf(lcdOutput, "< MoGo + Cones >");
				else if (currentCode == cones)	sprintf(lcdOutput, "<     Cones    >");
				else if (currentCode == mogo)	sprintf(lcdOutput, "<  Mobile Goal >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentMenu = allianceColorMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColor == red)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColor == blue)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentMenu = startingSideMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSide == leftSide)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSide == rightSide)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentMenu = mainMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				displayLCDCenteredString(1, "<      OK      >");
				break;
			}
		}
	}
	else lcdButtonsPressed[2] = false;

	if (nLCDButtons == 1) {
		if (!lcdButtonsPressed[0]) {
			lcdButtonsPressed[0] = true;
			switch (currentMenu) {
			case mainMenu:
				currentMenu = startingSideMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSide == leftSide)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSide == rightSide)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentMenu = allianceColorMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColor == red)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColor == blue)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentMenu = autonomousMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == mogoAndCones)	sprintf(lcdOutput, "< MoGo + Cones >");
				else if (currentCode == cones)	sprintf(lcdOutput, "<     Cones    >");
				else if (currentCode == mogo)	sprintf(lcdOutput, "<  Mobile Goal >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentMenu = powerExpanderMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Power Expander V");
				sprintf(lcdOutput, "<     %1.2f%s", SensorValue[POWER_EXPANDER_STATUS]/POWER_EXPANDER_DIVISOR,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case powerExpanderMenu:
				currentMenu = backupBatteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case backupBatteryVoltageMenu:
				currentMenu = batteryVoltageMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case batteryVoltageMenu:
				currentMenu = mainMenu;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				displayLCDCenteredString(1, "<      OK      >");
				break;
			}
		}
	}
	else lcdButtonsPressed[0] = false;

	if (nLCDButtons == 2) {
		if (!lcdButtonsPressed[1]) {
			lcdButtonsPressed[1] = true;
			switch (currentMenu) {
			case mainMenu:
				lcdReady = true;

				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDCenteredString(0, "     2223-Z     ");
				break;

			case batteryVoltageMenu:
				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Battery Voltage");
				sprintf(lcdOutput, "<     %1.2f%s", nImmediateBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;
			case backupBatteryVoltageMenu:
				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Backup Battery V");
				sprintf(lcdOutput, "<     %1.2f%s", BackupBatteryLevel/1000.0,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case powerExpanderMenu:
				clearLCDLine(0);
				clearLCDLine(1);

				displayLCDString(0, 0, "Power Expander V");
				sprintf(lcdOutput, "<     %1.2f%s", SensorValue[POWER_EXPANDER_STATUS]/POWER_EXPANDER_DIVISOR,"V    >");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case autonomousMenu:
				currentCode = (currentCode == cones) ? mogoAndCones :
				(currentCode == mogoAndCones) ? mogo :
				cones;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentCode == mogoAndCones)	sprintf(lcdOutput, "< MoGo + Cones >");
				else if (currentCode == cones)	sprintf(lcdOutput, "<     Cones    >");
				else if (currentCode == mogo)	sprintf(lcdOutput, "<  Mobile Goal >");

				displayLCDCenteredString(0, "Autonomous");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case allianceColorMenu:
				currentColor = currentColor == blue ? red : blue;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentColor == red)	sprintf(lcdOutput,  "<     Red      >");
				else if (currentColor == blue)	sprintf(lcdOutput, "<     Blue     >");

				displayLCDCenteredString(0, "Color");
				displayLCDCenteredString(1, lcdOutput);
				break;

			case startingSideMenu:
				currentSide = currentSide == leftSide ? rightSide : leftSide;

				clearLCDLine(0);
				clearLCDLine(1);

				if (currentSide == leftSide)	sprintf(lcdOutput,  "<     Left     >");
				else if (currentSide == rightSide)	sprintf(lcdOutput, "<     Right    >");

				displayLCDCenteredString(0, "Side");
				displayLCDCenteredString(1, lcdOutput);
				break;

			}
		}
	}
	else lcdButtonsPressed[1] = false;
}
#endif
#endif
