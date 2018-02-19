/*   functions.h - functions with which to control Model 6                  *
*    Copyright (C) <2017>  Marcos Ricardo Pesante Colón                     *
*                                                                           *
*    This program is free software: you can redistribute it and/or modify   *
*    it under the terms of the GNU General Public License as published by   *
*    the Free Software Foundation, either version 3 of the License, or      *
*    (at your option) any later version.                                    *
*                                                                           *
*    This program is distributed in the hope that it will be useful,        *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of         *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
*    GNU General Public License for more details.                           *
*                                                                           *
*    You should have received a copy of the GNU General Public License      *
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#ifndef functions.h
#define functions.h

#pragma systemFile

//Direction the drive will move
enum direction{ f, b, l, r};

//Target mechanism in the "hold()" function
enum target{ kDrive, kArm, kMoGo, kClaw, kAll};

//Which sensor will the Kalman Filter use to estimate it's true value
enum sensorTarget{ kDriveEncoderLeft, kDriveEncoderRight, kGyro, kArmPotentiometerLeft, kArmPotentiometerRight};

//What positon will the right claw be
enum clawPosition{ kOpen, kClose};

//Variables for PID
typedef struct{
	float kP;
	float kI;
	float kD;
	short error;
	short lastError;
	short integral;
	byte integralLimit;
	byte output;} PID;

//Later used for calculations
typedef struct{
	float width;
	float wheelDiameter;
	unsigned short rotationTicks;} baseConstants;
typedef struct{
	short maxArmHeight;
	short minArmHeight;
}armConstants;

//Kalman Filter values
typedef struct{
	float kg;
	float estimate;
	float previousEstimate;
	float errorEstimate;
	float previousErrorEstimate;
	byte errorMeasurement;} kalmanFilter;

//Declare structs
static baseConstants baseConstant;
static armConstants armConstant;
static PID drive;
static PID arm;
static kalmanFilter encoderLeft;
static kalmanFilter encoderRight;
static kalmanFilter gyro;
static kalmanFilter potentiometerLeft;
static kalmanFilter potentiometerRight;

static short button8RToggleState, driveThreshold, slewTime, slewIncrement, joystickVertical, joystickHorizontal, counter;
static bool driveDone = false, armDone = false, clawDone = false, button8RPressed, button6DPressed, button6UPressed, button5DPressed, button5UPressed;

static void initialize(){
	//Stop all motors
	motor[driveBL] = 0;
	motor[driveFL] = 0;
	motor[driveBR] = 0;
	motor[driveFR] = 0;
	motor[armL] = 0;
	motor[armR] = 0;
	motor[moGoL] = 0;
	motor[moGoR] = 0;
	motor[clawL] = 0;
	motor[clawR] = 0;

	baseConstant.width	= 16.5;
	baseConstant.wheelDiameter = 3.25;
	baseConstant.rotationTicks = 4000;																						//How many ticks does the gyro read on ONE full rotation
	armConstant.maxArmHeight = 1024;
	armConstant.minArmHeight = 0;

	//Define the PID constants for base
	drive.kP = 1.27;
	drive.kI = 0.025;
	drive.kD = 3;
	drive.integralLimit = 20;

	//Define the PID constants for arm
	arm.kP = 1.75;
	arm.kI = 0.025;
	arm.kD = 3;
	arm.integralLimit = 20;

	//Reset encoder values
	SensorValue[driveEncoderL] = 0;
	SensorValue[driveEncoderR] = 0;

	//Calibrate Gyro
	SensorType[in8] = sensorNone;
	wait1Msec(1000);
	SensorType[in8] = sensorGyro;
	wait1Msec(1500);

	//Kalman Filter Variable Initialization
	encoderLeft.kg = 0;
	encoderLeft.estimate=0;
	encoderLeft.previousEstimate = 0;
	encoderLeft.errorEstimate = 0;
	encoderLeft.previousErrorEstimate = 0;
	encoderLeft.errorMeasurement = 5;
	encoderRight.kg = 0;
	encoderRight.estimate=0;
	encoderRight.previousEstimate = 0;
	encoderRight.errorEstimate = 0;
	encoderRight.previousErrorEstimate = 0;
	encoderRight.errorMeasurement = 5;
	potentiometerLeft.kg = 0;
	potentiometerLeft.estimate=0;
	potentiometerLeft.previousEstimate = 0;
	potentiometerLeft.errorEstimate = 0;
	potentiometerLeft.previousErrorEstimate = 0;
	potentiometerLeft.errorMeasurement = 5;
	potentiometerRight.kg = 0;
	potentiometerRight.estimate=0;
	potentiometerRight.previousEstimate = 0;
	potentiometerRight.errorEstimate = 0;
	potentiometerRight.previousErrorEstimate = 0;
	potentiometerRight.errorMeasurement = 5;
	gyro.kg = 0;
	gyro.estimate=0;
	gyro.previousEstimate = 0;
	gyro.errorEstimate = 0;
	gyro.previousErrorEstimate = 0;
	gyro.errorMeasurement = 20;

	//Driver control variables
	button8RToggleState = 0;
	button8RPressed = false;
	button5UPressed = false;
	button5DPressed = false;
	button6UPressed = false;
	button6DPressed = false;
	driveThreshold = 15;
	slewTime = 15;
	slewIncrement = 15;
	joystickVertical = 0;
	joystickHorizontal = 0;

	//Counter
	counter = 0;
}

static void resetValues(){
	//Stop all motors
	motor[driveBL] = 0;
	motor[driveFL] = 0;
	motor[driveBR] = 0;
	motor[driveFR] = 0;
	motor[armL] = 0;
	motor[armR] = 0;
	motor[moGoL] = 0;
	motor[moGoR] = 0;
	motor[clawL] = 0;
	motor[clawR] = 0;

	//Reset encoder values
	SensorValue[driveEncoderL] = 0;
	SensorValue[driveEncoderR] = 0;

	//Reset Kalman Filter Variables
	encoderLeft.kg = 0;
	encoderLeft.estimate=0;
	encoderLeft.previousEstimate = 0;
	encoderLeft.errorEstimate = 0;
	encoderLeft.previousErrorEstimate = 0;
	encoderLeft.errorMeasurement = 5;
	encoderRight.kg = 0;
	encoderRight.estimate=0;
	encoderRight.previousEstimate = 0;
	encoderRight.errorEstimate = 0;
	encoderRight.previousErrorEstimate = 0;
	encoderRight.errorMeasurement = 5;
	potentiometerLeft.kg = 0;
	potentiometerLeft.estimate=0;
	potentiometerLeft.previousEstimate = 0;
	potentiometerLeft.errorEstimate = 0;
	potentiometerLeft.previousErrorEstimate = 0;
	potentiometerLeft.errorMeasurement = 5;
	potentiometerRight.kg = 0;
	potentiometerRight.estimate=0;
	potentiometerRight.previousEstimate = 0;
	potentiometerRight.errorEstimate = 0;
	potentiometerRight.previousErrorEstimate = 0;
	potentiometerRight.errorMeasurement = 5;
	gyro.kg = 0;
	gyro.estimate=0;
	gyro.previousEstimate = 0;
	gyro.errorEstimate = 0;
	gyro.previousErrorEstimate = 0;
	gyro.errorMeasurement = 5;

	//Reset PID variables for base
	drive.error = 0;
	drive.lastError = 0;
	drive.integral = 0;
	drive.integralLimit = 0;
	drive.output = 0;

	//Reset PID variables for base
	arm.error = 0;
	arm.lastError = 0;
	arm.integral = 0;
	arm.integralLimit = 0;
	arm.output = 0;

	driveDone = false;
	armDone = false;
	clawDone = false;
	counter = 0;
}

//This function serves also as a "stop"
static void hold(target subs, byte speed){
	switch(subs){

	case kDrive:
		motor[driveFL] = speed;
		motor[driveBL] = speed;
		motor[driveFR] = speed;
		motor[driveBR] = speed;
		break;

	case kArm:
		motor[armL] = speed;
		motor[armR] = speed;
		break;

	case kMoGo:
		motor[moGoL] = speed;
		motor[moGoR] = speed;
		break;

	case kClaw:
		motor[clawL] = speed;
		motor[clawR] = speed;
		break;

	case kAll:
		motor[driveBL] = speed;
		motor[driveFL] = speed;
		motor[driveBR] = speed;
		motor[driveFR] = speed;
		motor[armL] = speed;
		motor[armR] = speed;
		motor[moGoL] = speed;
		motor[moGoR] = speed;
		motor[clawL] = speed;
		motor[clawR] = speed;
		break;
	}
}

//Credit to ilectureonline.com
static int getSensor(sensorTarget sensor){
	switch(sensor){

	case kDriveEncoderLeft:
		//Calculate Kalman Gain, Estimate and error in estimate
		encoderLeft.kg = encoderLeft.errorEstimate / (encoderLeft.errorEstimate - encoderLeft.errorMeasurement);
		encoderLeft.estimate = encoderLeft.previousEstimate + encoderLeft.kg *(SensorValue[driveEncoderL]-encoderLeft.previousEstimate);
		encoderLeft.errorEstimate = (1 - encoderLeft.kg) * encoderLeft.previousErrorEstimate;

		//Prepare previous values for next loop
		encoderLeft.previousEstimate = encoderLeft.estimate;
		encoderLeft.previousErrorEstimate = encoderLeft.errorEstimate;

		return(encoderLeft.estimate);

	case kDriveEncoderRight:
		//Calculate Kalman Gain, Estimate and error in estimate
		encoderRight.kg = encoderRight.errorEstimate / (encoderRight.errorEstimate - encoderRight.errorMeasurement);
		encoderRight.estimate = encoderRight.previousEstimate + encoderRight.kg *(SensorValue[driveEncoderR]-encoderRight.previousEstimate);
		encoderRight.errorEstimate = (1 - encoderRight.kg) * encoderRight.previousErrorEstimate;

		//Prepare previous values for next loop
		encoderRight.previousEstimate = encoderRight.estimate;
		encoderRight.previousErrorEstimate = encoderRight.errorEstimate;

		return(encoderRight.estimate);

	case kGyro:
		//Calculate Kalman Gain, Estimate and error in estimate
		gyro.kg = gyro.errorEstimate / (gyro.errorEstimate - gyro.errorMeasurement);
		gyro.estimate = gyro.previousEstimate + gyro.kg *(SensorValue[in8]-gyro.previousEstimate);
		gyro.errorEstimate = (1 - gyro.kg) * gyro.previousErrorEstimate;

		//Prepare previous values for next loop
		gyro.previousEstimate = gyro.estimate;
		gyro.previousErrorEstimate = gyro.errorEstimate;

		return(gyro.estimate);

	case kArmPotentiometerLeft:
		//Calculate Kalman Gain, Estimate and error in estimate
		potentiometerLeft.kg = potentiometerLeft.errorEstimate / (potentiometerLeft.errorEstimate - potentiometerLeft.errorMeasurement);
		potentiometerLeft.estimate = potentiometerLeft.previousEstimate + potentiometerLeft.kg *(SensorValue[armPotL]-potentiometerLeft.previousEstimate);
		potentiometerLeft.errorEstimate = (1 - potentiometerLeft.kg) * potentiometerLeft.previousErrorEstimate;

		//Prepare previous values for next loop
		potentiometerLeft.previousEstimate = potentiometerLeft.estimate;
		potentiometerLeft.previousErrorEstimate = potentiometerLeft.errorEstimate;

		return(potentiometerLeft.estimate);

	case kArmPotentiometerRight:
		//Calculate Kalman Gain, Estimate and error in estimate
		potentiometerRight.kg = potentiometerRight.errorEstimate / (potentiometerRight.errorEstimate - potentiometerRight.errorMeasurement);
		potentiometerRight.estimate = potentiometerRight.previousEstimate + potentiometerRight.kg *(SensorValue[armPotR]-potentiometerRight.previousEstimate);
		potentiometerRight.errorEstimate = (1 - potentiometerRight.kg) * potentiometerRight.previousErrorEstimate;

		//Prepare previous values for next loop
		potentiometerRight.previousEstimate = potentiometerRight.estimate;
		potentiometerRight.previousErrorEstimate = potentiometerRight.errorEstimate;

		return(potentiometerRight.estimate);
	}
	return(0);
}

static byte withinRange(int input){
	byte value = input>127 ? 127 : input;
	value = value<-127 ? -127 : value;
	return(value);
}

static int inchesToPulses(float inches){
	return(ceil((360/baseConstant.wheelDiameter*3.1415926535897932384626433832795)*inches));
}

static int degreesToPulses(short angle, bool useGyro = true){
	if(useGyro){
		//Convert degrees rotation to pulses
		return(ceil(angle *(baseConstant.rotationTicks/360)));
	}
	else{
		//Convert degrees of rotation to pulses using the circle's arc formula
		return(floor(((angle/360) * baseConstant.width * 3.1415926535897932384626433832795)/(baseConstant.wheelDiameter / 360)));
	}
}

static void move(direction side, float distance, byte speed, bool useGyro = true){
	//Recalculate distance
	if(side == l || side == r){
		distance = degreesToPulses(distance, useGyro);
	}
	else{
		//Convert the distance from inches to pulses
		distance = inchesToPulses(distance);
	}

	//PID
	drive.error = distance - ((abs(getSensor(kDriveEncoderLeft)) + abs(getSensor(kDriveEncoderRight)))/2);
	if(drive.error != 0 && drive.integral < drive.integralLimit && drive.integral > -drive.integralLimit) drive.integral += drive.error;
	else drive.integral = 0;

	drive.output = withinRange((drive.kP * drive.error) + (drive.kI * drive.integral) + (drive.kD * (drive.error != 0 ? drive.error - drive.lastError : 0)));

	drive.lastError = drive.error;

	//Make sure output is between 127 and -127 and rectify robot if necessary
	byte leftOutput = withinRange(drive.output / ((abs(getSensor(kDriveEncoderLeft)) - abs(getSensor(kDriveEncoderRight))) * 0.01 + 1));
	byte rightOutput = withinRange(drive.output / ((abs(getSensor(kDriveEncoderRight)) - abs(getSensor(kDriveEncoderLeft))) * 0.01 + 1));

	//Move motors based on PID values and direction in which to move
	switch(side){
	case f:
		motor[driveFL] = leftOutput;
		motor[driveBL] = leftOutput;
		motor[driveFR] = rightOutput;
		motor[driveBR] = rightOutput;
		break;

	case b:
		motor[driveFL] = -leftOutput;
		motor[driveBL] = -leftOutput;
		motor[driveFR] = -rightOutput;
		motor[driveBR] = -rightOutput;
		break;

	case l:
		motor[driveFL] = -leftOutput;
		motor[driveBL] = -leftOutput;
		motor[driveFR] = rightOutput;
		motor[driveBR] = rightOutput;
		break;

	case r:
		motor[driveFL] = leftOutput;
		motor[driveBL] = leftOutput;
		motor[driveFR] = -rightOutput;
		motor[driveBR] = -rightOutput;
		break;

	}

	//Look if process is finished with a threshold of +-10
	if((getSensor(kDriveEncoderLeft)+getSensor(kDriveEncoderRight))/2 < distance+10 && (getSensor(kDriveEncoderLeft)+getSensor(kDriveEncoderRight))/2 > distance-10) driveDone = true;
}

static void lift(bool position){

	if(position == 1){
		//PID
		arm.error = armConstant.maxArmHeight - ((getSensor(kArmPotentiometerLeft) + getSensor(kArmPotentiometerRight))/2);
		if(arm.error != 0 && arm.integral < arm.integralLimit && arm.integral > -arm.integralLimit) arm.integral += arm.error;
		else arm.integral = 0;

		arm.output = withinRange((arm.kP * arm.error) + (arm.kI * arm.integral) + (arm.kD * (arm.error != 0 ? arm.error - arm.lastError : 0)));

		arm.lastError = arm.error;

		//Look if process is finished with a threshold of +-10
		if(((getSensor(kArmPotentiometerLeft) + getSensor(kArmPotentiometerRight)) / 2) < (armConstant.maxArmHeight + 10) && ((getSensor(kArmPotentiometerLeft)+getSensor(kArmPotentiometerRight))/2) > (armConstant.maxArmHeight-10)) armDone = true;

		}else{
		//PID
		arm.error = armConstant.minArmHeight - ((getSensor(kArmPotentiometerLeft) + getSensor(kArmPotentiometerRight))/2);
		if(arm.error != 0 && arm.integral < arm.integralLimit && arm.integral > -arm.integralLimit) arm.integral += arm.error;
		else arm.integral = 0;

		arm.output = withinRange((arm.kP * arm.error) + (arm.kI * arm.integral) + (arm.kD * (arm.error != 0 ? arm.error - arm.lastError : 0)));

		arm.lastError = arm.error;

		//Look if process is finished with a threshold of +-10
		if(((getSensor(kArmPotentiometerLeft) + getSensor(kArmPotentiometerRight)) / 2) < (armConstant.minArmHeight + 10) && ((getSensor(kArmPotentiometerLeft)+getSensor(kArmPotentiometerRight))/2) > (armConstant.minArmHeight-10)) armDone = true;
	}

	//Make sure output is between 127 and -127 and rectify arms if necessary
	byte leftOutput = withinRange(arm.output / ((getSensor(kArmPotentiometerLeft) - getSensor(kArmPotentiometerRight)) * 0.01 + 1));
	byte rightOutput = withinRange(arm.output / ((getSensor(kArmPotentiometerRight) - getSensor(kArmPotentiometerLeft)) * 0.01 + 1));

	//Move motors based on PID values and direction in which to move
	motor[armL] = leftOutput;
	motor[armR] = rightOutput;
}

static void claw(clawPosition position){

	//Open the right claw and close the left claw
	if(position == kOpen){
		motor[clawL] = 100;
		motor[clawR] = 100;
	}

	//Close the right claw and open the left claw
	else if(position == kClose){
		motor[clawL] = -100;
		motor[clawR] = -100;
	}

	//If 0.5 seconds passed, stop motors
	if(counter >= 33){
		motor[clawL] = 0;
		motor[clawR] = 0;
		clawDone = true;
	}
}

/****************************************************************--Driver control--*****************************************************************/

void normalDrive(byte x, byte y){
	//make sure small movements of the joystick do not move the robot
	motor[driveBL] = (x > driveThreshold  || x < -driveThreshold || y > driveThreshold || y < -driveThreshold) ? x - y  : 0;
	motor[driveFL] = (x > driveThreshold  || x < -driveThreshold || y > driveThreshold || y < -driveThreshold) ? x - y  : 0;
	motor[driveBR] = (x > driveThreshold  || x < -driveThreshold || y > driveThreshold || y < -driveThreshold) ? x + y  : 0;
	motor[driveFR] = (x > driveThreshold  || x < -driveThreshold || y > driveThreshold || y < -driveThreshold) ? x + y  : 0;
}
void invertedDirectionDrive(byte x, byte y){
	//make sure small movements of the joystick do not move the robot
	motor[driveBL] = (x > driveThreshold  || x < -driveThreshold || y > driveThreshold || y < -driveThreshold) ? -x - y  : 0;
	motor[driveFL] = (x > driveThreshold  || x < -driveThreshold || y > driveThreshold || y < -driveThreshold) ? -x - y  : 0;
	motor[driveBR] = (x > driveThreshold  || x < -driveThreshold || y > driveThreshold || y < -driveThreshold) ? -x + y  : 0;
	motor[driveFR] = (x > driveThreshold  || x < -driveThreshold || y > driveThreshold || y < -driveThreshold) ? -x + y  : 0;
}

static void driverControl(){
	//Slew rate control (basically accelerates and deaccelerates before changing speeds too quickly to preserve motor life)

	if(vexRT[Ch2] > 0 && vexRT[Ch2] > joystickHorizontal) joystickHorizontal += slewIncrement;
	else if (vexRT[Ch2] < 0 && vexRT[Ch2] < joystickHorizontal ) joystickHorizontal-= slewIncrement;
	else if (vexRT[Ch2] == 0 && joystickHorizontal < 0) joystickHorizontal += slewIncrement;
	else if (vexRT[Ch2] == 0 && joystickHorizontal > 0) joystickHorizontal -= slewIncrement;

	if(vexRT[Ch1] > 0 && joystickVertical < vexRT[Ch1]) joystickVertical += slewIncrement;
	else if (vexRT[Ch1] < 0 && joystickVertical < vexRT[Ch1]) joystickVertical-= slewIncrement;
	else if (vexRT[Ch1] == 0 && joystickVertical < 0) joystickVertical += slewIncrement;
	else if (vexRT[Ch1] == 0 && joystickVertical > 0) joystickVertical -= slewIncrement;

	//Drive
	if(vexRT[Btn8R]){
		if(!button8RPressed){
			button8RToggleState = 1 - button8RToggleState;		//Change the toggle state
			button8RPressed = true;															//ButtonPressed = 1 so the toggle state doesnt change again
		}
	}
	else button8RPressed = false;															//The button is not pressed so buttonPressed = 0

	if(button8RToggleState)	invertedDirectionDrive(joystickHorizontal, joystickVertical);
	else	normalDrive(joystickHorizontal, joystickVertical);

	//Arm
	if(vexRT[Btn6D]){
		if(!button6DPressed){
			lift(0);
			button6DPressed = true;
		}
	}
	else button6DPressed = false;

	if(vexRT[Btn6U]){
		if(!button6UPressed){
			lift(1);
			button6UPressed = true;
		}
	}
	else button6UPressed = false;

	//Claw
	if(vexRT[Btn5D]){
		if(!button5DPressed){
			claw(kOpen);
			button5DPressed = true;
		}
	}
	else button5DPressed = false;

	if(vexRT[Btn5U]){
		if(!button5UPressed){
			claw(kClose);
			button5UPressed = true;
		}
	}
	else button5UPressed = false;


	wait1Msec(slewTime);
}
#endif
