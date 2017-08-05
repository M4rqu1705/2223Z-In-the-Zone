#ifndef functions.h
#define functions.h

#pragma systemFile

//Enum to determine the direction the drive will move
enum direction{ f, b, l, r};

//Enum to determine the target mechanism in the "hold()" function
enum target{ kDrive, kArm, kMoGo, kClaw, kAll};

//Enum to determine which sensor will the Kalman Filter use to estimate it's true value
enum sensorTarget{ kDriveEncoderLeft, kDriveEncoderRight, kGyro, kArmPotentiometerLeft, kArmPotentiometerRight};

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

static int button8RToggleState, button8RPressed, driveThreshold, slewTime, joystickVertical, joystickHorizontal;
static bool driveDone = false, armDone = false, clawDone = false;

static void initialize();

static void resetValues();

static byte withinRange(int input){
	byte value = input>127 ? 127 : input;
	value = value<-127 ? -127 : value;
	return(value);
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

static void move(direction side, float distance, byte speed, bool useGyro = true){
	//Determine if we have to recalculate the distance
	if(side == l || side == r){
		if(useGyro){
			distance = ceil(distance *(baseConstant.rotationTicks/360));	//Calculate distance for use with Gyro
			}	else{
			distance = floor(((distance/360) * baseConstant.width * 3.1415926535897932384626433832795)/(baseConstant.wheelDiameter / 360));	//Calculate distance for use with Encoders
		}
		} else{
		distance = ceil((360/baseConstant.wheelDiameter*3.1415926535897932384626433832795)*distance);
	}

	//PID
	//Calculate error
	drive.error = distance - ((abs(getSensor(kDriveEncoderLeft)) + abs(getSensor(kDriveEncoderRight)))/2);
	//Calculate integral if within range and error != 0
	if(drive.error != 0 && drive.integral < drive.integralLimit && drive.integral > -drive.integralLimit) drive.integral += drive.error;
	else drive.integral = 0;
	//Calculate the output multiplying the constants by the error, integral and derivative
	drive.output = withinRange((drive.kP * drive.error) + (drive.kI * drive.integral) + (drive.kD * (drive.error != 0 ? drive.error - drive.lastError : 0)));
	//Prepare value for next loop
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
	//If position == 1, then move the arms to specific position
	if(position){
		//PID
		//Calculate error
		arm.error = armConstant.maxArmHeight - ((getSensor(kArmPotentiometerLeft) + getSensor(kArmPotentiometerRight))/2);
		//Calculate integral if within range and error != 0
		if(arm.error != 0 && arm.integral < arm.integralLimit && arm.integral > -arm.integralLimit) arm.integral += arm.error;
		else arm.integral = 0;
		//Calculate the output multiplying the constants by the error, integral and derivative
		arm.output = withinRange((arm.kP * arm.error) + (arm.kI * arm.integral) + (arm.kD * (arm.error != 0 ? arm.error - arm.lastError : 0)));
		//Prepare value for next loop
		arm.lastError = arm.error;
	}
	//If position == 0, then move arms to opposite position
	else{
		//PID
		//Calculate error
		arm.error = armConstant.minArmHeight - ((getSensor(kArmPotentiometerLeft) + getSensor(kArmPotentiometerRight))/2);
		//Calculate integral if within range and error != 0
		if(arm.error != 0 && arm.integral < arm.integralLimit && arm.integral > -arm.integralLimit) arm.integral += arm.error;
		else arm.integral = 0;
		//Calculate the output multiplying the constants by the error, integral and derivative
		arm.output = withinRange((arm.kP * arm.error) + (arm.kI * arm.integral) + (arm.kD * (arm.error != 0 ? arm.error - arm.lastError : 0)));
		//Prepare value for next loop
		arm.lastError = arm.error;
	}
	//Make sure output is between 127 and -127 and rectify robot if necessary
	byte leftOutput = withinRange(arm.output / ((getSensor(kArmPotentiometerLeft) - getSensor(kArmPotentiometerRight)) * 0.01 + 1));
	byte rightOutput = withinRange(arm.output / ((getSensor(kArmPotentiometerRight) - getSensor(kArmPotentiometerLeft)) * 0.01 + 1));

	//Move motors based on PID values and direction in which to move
	motor[armL] = leftOutput;
	motor[armR] = rightOutput;

	if((getSensor(kArmPotentiometerLeft)+getSensor(kArmPotentiometerRight))/2 < armConstant.minArmHeight+10 && (getSensor(kArmPotentiometerLeft)+getSensor(kArmPotentiometerRight))/2 > armConstant.minArmHeight-10) armDone = true;

}

static void intake(unsigned short ticks, byte speed){
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

	if(vexRT[Ch2] >= 0 && joystickHorizontal <= vexRT[Ch2]) joystickHorizontal++;
	else if (vexRT[Ch2] < 0 && joystickHorizontal <= vexRT[Ch2]) joystickHorizontal--;

	if(vexRT[Ch1] > 0 && joystickVertical <= vexRT[Ch1]) joystickVertical++;
	else if (vexRT[Ch1] < 0 && joystickVertical <= vexRT[Ch1]) joystickVertical--;

	//Drive
	if( vexRT[Btn8R] == 1 ){
		if(!button8RPressed){																//if buttonPressed was previously 0 when Btn8R pressed then
			button8RToggleState = 1 - button8RToggleState;		// change the toggle state
			button8RPressed = 1;															// buttonPressed = 1 so the toggle state doesnt change again
		}
	}
	else button8RPressed = 0;															// the button is not pressed so buttonPressed = 0

	if(button8RToggleState)	invertedDirectionDrive(joystickHorizontal, joystickVertical);
	else	normalDrive(joystickHorizontal, joystickVertical);

	//Arm

	//XBar

	//Claw

	wait1Msec(slewTime);
}

static void initialize(){
	baseConstant.width	= 16.5;
	baseConstant.wheelDiameter = 3.25;
	baseConstant.rotationTicks= 4000;																						//How many ticks does the gyro read on ONE full rotation

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
	button8RPressed = 0;
	driveThreshold = 20;
	slewTime = 1;
	joystickVertical = 0;
	joystickHorizontal = 0;
}

static void resetValues(){
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
}

#endif
