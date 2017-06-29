#ifndef functions.h
#define functions.h

#pragma systemFile

//Enum to determine the direction the drive will move
enum direction{ f, b, l, r};

//Enum to determine the target mechanism in the "hold()" function
enum target{ kBase, kArm, kXBar, kClaw, kAll};

//Enum to determine which sensor will the Kalman Filter use to estimate it's true value
enum sensorTarget{ kEncoderLeft, kEncoderRight, kGyro, kPotentiometerArm, kPotentiometerXBar};

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
	unsigned short rotationTicks;} baseValues;
typedef struct{
	unsigned short maxPotHeight;
	unsigned short minPotHeight;
	float maxArmHeight;
	float minArmHeight;} armValues;

//Kalman Filter values
typedef struct{
	float kg;
	float estimate;
	float previousEstimate;
	float errorEstimate;
	float previousEstimate;
	float previousErrorEstimate;
	byte errorMeasurement;} kalmanFilter;

//Declare structs
static baseValues baseConstant;
static armValues armConstant;
static PID drive;
static PID arm;
static kalmanFilter encoderLeft;
static kalmanFilter encoderRight;
static kalmanFilter gyro;
static kalmanFilter potentiometerArm;
static kalmanFilter potentiometerXBar;

static int button8RToggleState, button8RPressed, driveThreshold;
static bool baseDone = false, armDone = false, clawDone = false;

static void initialize(){
	baseConstant.width	= 16.5;
	baseConstant.wheelDiameter = 3.25;
	baseConstant.rotationTicks= 4000;																						//How many ticks does the gyro read on ONE full rotation
	armConstant.maxPotHeight = 1024;																						//Since the potentiometer isn't used in its full spectrum, put max value
	armConstant.minPotHeight = 500;																							//Since the potentiometer isn't used in its full spectrum, put min value
	armConstant.maxArmHeight = 47.5;																						//Put arm's max height (used later for calculations)
	armConstant.minArmHeight = 17.5;																						//Put arm's min height (used later for calculations)

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
	SensorValue[LeftEncoder] = 0;
	SensorValue[RightEncoder] = 0;

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
	encoderLeft.previousEstimate = 0;
	encoderLeft.previousErrorEstimate = 0;
	encoderLeft.errorMeasurement = 5;
	encoderRight.kg = 0;
	encoderRight.estimate=0;
	encoderRight.previousEstimate = 0;
	encoderRight.errorEstimate = 0;
	encoderRight.previousEstimate = 0;
	encoderRight.previousErrorEstimate = 0;
	encoderRight.errorMeasurement = 5;
	potentiometerArm.kg = 0;
	potentiometerArm.estimate=0;
	potentiometerArm.previousEstimate = 0;
	potentiometerArm.errorEstimate = 0;
	potentiometerArm.previousEstimate = 0;
	potentiometerArm.previousErrorEstimate = 0;
	potentiometerArm.errorMeasurement = 5;
	potentiometerXBar.kg = 0;
	potentiometerXBar.estimate=0;
	potentiometerXBar.previousEstimate = 0;
	potentiometerXBar.errorEstimate = 0;
	potentiometerXBar.previousEstimate = 0;
	potentiometerXBar.previousErrorEstimate = 0;
	potentiometerXBar.errorMeasurement = 5;
	gyro.kg = 0;
	gyro.estimate=0;
	gyro.previousEstimate = 0;
	gyro.errorEstimate = 0;
	gyro.previousEstimate = 0;
	gyro.previousErrorEstimate = 0;
	gyro.errorMeasurement = 20;

	//Driver control variables
	button8RToggleState = 0;
	button8RPressed = 0;
	driveThreshold = 20;
}

static void resetValues(){
	//Reset encoder values
	SensorValue[LeftEncoder] = 0;
	SensorValue[RightEncoder] = 0;

	//Reset Kalman Filter Variables
	encoderLeft.kg = 0;
	encoderLeft.estimate=0;
	encoderLeft.previousEstimate = 0;
	encoderLeft.errorEstimate = 0;
	encoderLeft.previousEstimate = 0;
	encoderLeft.previousErrorEstimate = 0;
	encoderLeft.errorMeasurement = 5;
	encoderRight.kg = 0;
	encoderRight.estimate=0;
	encoderRight.previousEstimate = 0;
	encoderRight.errorEstimate = 0;
	encoderRight.previousEstimate = 0;
	encoderRight.previousErrorEstimate = 0;
	encoderRight.errorMeasurement = 5;
	potentiometerArm.kg = 0;
	potentiometerArm.estimate=0;
	potentiometerArm.previousEstimate = 0;
	potentiometerArm.errorEstimate = 0;
	potentiometerArm.previousEstimate = 0;
	potentiometerArm.previousErrorEstimate = 0;
	potentiometerArm.errorMeasurement = 5;
	potentiometerXBar.kg = 0;
	potentiometerXBar.estimate=0;
	potentiometerXBar.previousEstimate = 0;
	potentiometerXBar.errorEstimate = 0;
	potentiometerXBar.previousEstimate = 0;
	potentiometerXBar.previousErrorEstimate = 0;
	potentiometerXBar.errorMeasurement = 5;
	gyro.kg = 0;
	gyro.estimate=0;
	gyro.previousEstimate = 0;
	gyro.errorEstimate = 0;
	gyro.previousEstimate = 0;
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

	baseDone = false;
	armDone = false;
	clawDone = false;
}

//This function serves also as a "stop"
static void hold(target subs, byte speed){
	switch(subs){

	case kBase:
		motor[driveLeftBack] = speed;
		motor[driveLeftFront] = speed;
		motor[driveRightBack] = speed;
		motor[driveRightFront] = speed;
		break;

	case kArm:
		motor[liftLeft] = speed;
		motor[liftRight] = speed;
		break;

	case kXBar:
		motor[xBarLeft] = speed;
		motor[xBarRight] = speed;
		break;

	case kClaw:
		motor[clawMotor] = speed;
		break;

	case kAll:
		motor[driveLeftBack] = speed;
		motor[driveLeftFront] = speed;
		motor[driveRightBack] = speed;
		motor[driveRightFront] = speed;
		motor[liftLeft] = speed;
		motor[liftRight] = speed;
		motor[xBarLeft] = speed;
		motor[xBarRight] = speed;
		motor[clawMotor] = speed;
		break;
	}
}

//Credit to ilectureonline.com
static int getSensor(sensorTarget sensor){
	switch(sensor){

	case kEncoderLeft:
		//Calculate Kalman Gain, Estimate and error in estimate
		encoderLeft.kg = encoderLeft.errorEstimate / (encoderLeft.errorEstimate - encoderLeft.errorMeasurement);
		encoderLeft.estimate = encoderLeft.previousEstimate + encoderLeft.kg *(SensorValue[LeftEncoder]-encoderLeft.previousEstimate);
		encoderLeft.errorEstimate = (1 - encoderLeft.kg) * encoderLeft.previousErrorEstimate;

		//Prepare previous values for next loop
		encoderLeft.previousEstimate = encoderLeft.estimate;
		encoderLeft.previousErrorEstimate = encoderLeft.errorEstimate;

		return(encoderLeft.estimate);

	case kEncoderRight:
		//Calculate Kalman Gain, Estimate and error in estimate
		encoderRight.kg = encoderRight.errorEstimate / (encoderRight.errorEstimate - encoderRight.errorMeasurement);
		encoderRight.estimate = encoderRight.previousEstimate + encoderRight.kg *(SensorValue[RightEncoder]-encoderRight.previousEstimate);
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

	case kPotentiometerArm:
		//Calculate Kalman Gain, Estimate and error in estimate
		potentiometerArm.kg = potentiometerArm.errorEstimate / (potentiometerArm.errorEstimate - potentiometerArm.errorMeasurement);
		potentiometerArm.estimate = potentiometerArm.previousEstimate + potentiometerArm.kg *(SensorValue[armPot]-potentiometerArm.previousEstimate);
		potentiometerArm.errorEstimate = (1 - potentiometerArm.kg) * potentiometerArm.previousErrorEstimate;

		//Prepare previous values for next loop
		potentiometerArm.previousEstimate = potentiometerArm.estimate;
		potentiometerArm.previousErrorEstimate = potentiometerArm.errorEstimate;

		return(potentiometerArm.estimate);

	case kPotentiometerXBar:
		//Calculate Kalman Gain, Estimate and error in estimate
		potentiometerXBar.kg = potentiometerXBar.errorEstimate / (potentiometerXBar.errorEstimate - potentiometerXBar.errorMeasurement);
		potentiometerXBar.estimate = potentiometerXBar.previousEstimate + potentiometerXBar.kg *(SensorValue[xBarPot]-potentiometerXBar.previousEstimate);
		potentiometerXBar.errorEstimate = (1 - potentiometerXBar.kg) * potentiometerXBar.previousErrorEstimate;

		//Prepare previous values for next loop
		potentiometerXBar.previousEstimate = potentiometerXBar.estimate;
		potentiometerXBar.previousErrorEstimate = potentiometerXBar.errorEstimate;

		return(potentiometerXBar.estimate);
	}
	return(0);
}

static void move(direction side, unsigned short pulses, byte speed, bool useGyro = true){
	//Determine if we have to recalculate the pulses
	if(side == l || side == r){
		if(useGyro){
			pulses = ceil(pulses *(baseConstant.rotationTicks/360));	//Calculate pulses for use with Gyro
		}
		else{
			pulses = floor(((pulses/360) * baseConstant.width * 3.1415926535897932384626433832795)/(baseConstant.wheelDiameter / 360));	//Calculate pulses for use with Encoders
		}
	}

	//PID
	//Calculate error
	drive.error = pulses - ((abs(getSensor(kEncoderLeft)) + abs(getSensor(kEncoderRight)))/2);
	//Calculate integral if within range and error != 0
	if(drive.error != 0 && drive.integral < drive.integralLimit && drive.integral > -drive.integralLimit) drive.integral += drive.error;
	else drive.integral = 0;
	//Calculate the output multiplying the constants by the error, integral and derivative
	drive.output = (drive.kP * drive.error) + (drive.kI * drive.integral) + (drive.kD * (drive.error != 0 ? drive.error - drive.lastError : 0));
	//Prepare value for next loop
	drive.lastError = drive.error;

	//Make sure output is between 127 and -127
	short leftOutput = drive.output / ((abs(getSensor(kEncoderLeft)) - abs(getSensor(kEncoderRight))) * 0.01 + 1);
	leftOutput = leftOutput > 127 ? 127: leftOutput;
	leftOutput = leftOutput < -127 ? -127:leftOutput;
	short rightOutput = drive.output / ((abs(getSensor(kEncoderRight)) - abs(getSensor(kEncoderLeft))) * 0.01 + 1);
	rightOutput = rightOutput > 127 ? 127: rightOutput;
	rightOutput = rightOutput < -127 ? -127:rightOutput;

	//Move motors based on PID values, direction in which to move, and "rectify" robot if necessary
	switch(side){
	case f:
		motor[driveLeftFront] = leftOutput;
		motor[driveLeftBack] = leftOutput;
		motor[driveRightFront] = rightOutput;
		motor[driveRightBack] = rightOutput;
		break;

	case b:
		motor[driveLeftFront] = -leftOutput;
		motor[driveLeftBack] = -leftOutput;
		motor[driveRightFront] = -rightOutput;
		motor[driveRightBack] = -rightOutput;
		break;

	case l:
		motor[driveLeftFront] = -leftOutput;
		motor[driveLeftBack] = -leftOutput;
		motor[driveRightFront] = rightOutput;
		motor[driveRightBack] = rightOutput;
		break;

	case r:
		motor[driveLeftFront] = leftOutput;
		motor[driveLeftBack] = leftOutput;
		motor[driveRightFront] = -rightOutput;
		motor[driveRightBack] = -rightOutput;
		break;

	}

	//Look if process is finished with a threshold of +-10
	if((getSensor(kEncoderLeft)+getSensor(kEncoderRight))/2 < pulses+10 && (getSensor(kEncoderLeft)+getSensor(kEncoderRight))/2 > pulses-10) baseDone = true;
}

static void lift(float height, byte speed){
	//Check if variable height exeeds the physical limits of the robot
	height = height > armConstant.maxArmHeight ? armConstant.maxArmHeight : height;
	height = height < armConstant.minArmHeight ? armConstant.minArmHeight : height;

	//Credit to https://www.arduino.cc/en/Reference/Map
	short ticks = floor((height - armConstant.minArmHeight) * (armConstant.maxPotHeight - armConstant.minPotHeight) / (armConstant.maxArmHeight - armConstant.minArmHeight) + armConstant.minPotHeight);

	//PID
	//Calculate error
	arm.error = ticks - getSensor(kPotentiometerArm);
	//Calculate integral if within range and error != 0
	if(arm.error != 0 && arm.integral < arm.integralLimit && arm.integral > -arm.integralLimit) arm.integral += arm.error;
	else arm.integral = 0;
	//Calculate the output multiplying the constants by the error, integral and derivative
	arm.output = (arm.kP * arm.error) + (arm.kI * arm.integral) + (arm.kD * (arm.error != 0 ? arm.error - arm.lastError : 0));
	//Prepare value for next loop
	arm.lastError = arm.error;

	//Move motors. Directon doesn't matter (PID will take care of that)
	motor[liftLeft] = arm.output;
	motor[liftRight] = arm.output;

	//Look if process is finished  with a threshold of +-10
	if(getSensor(kPotentiometerArm) < ticks+10 && getSensor(kPotentiometerArm) > ticks-10) armDone = true;
}

static void intake(unsigned short ticks, byte speed){
}


/****************************************************************--Driver control--*****************************************************************/

void normalDrive(){
	//make sure small movements of the joystick do not move the robot
	motor[driveLeftBack] = (vexRT[Ch2]>driveThreshold  || vexRT[Ch2]< -driveThreshold || vexRT[Ch1]>driveThreshold || vexRT[Ch1]< -driveThreshold) ? vexRT[Ch2] - vexRT[Ch1] : 0;
	motor[driveLeftFront] = (vexRT[Ch2]>driveThreshold  || vexRT[Ch2]< -driveThreshold || vexRT[Ch1]>driveThreshold || vexRT[Ch1]< -driveThreshold) ? vexRT[Ch2] - vexRT[Ch1] : 0;
	motor[driveRightBack] = (vexRT[Ch2]>driveThreshold  || vexRT[Ch2]< -driveThreshold || vexRT[Ch1]>driveThreshold || vexRT[Ch1]< -driveThreshold) ? vexRT[Ch2] + vexRT[Ch1] : 0;
	motor[driveRightFront] = (vexRT[Ch2]>driveThreshold  || vexRT[Ch2]< -driveThreshold || vexRT[Ch1]>driveThreshold || vexRT[Ch1]< -driveThreshold) ? vexRT[Ch2] + vexRT[Ch1] : 0;
}
void invertedDirectionDrive(){
	//make sure small movements of the joystick do not move the robot
	motor[driveLeftBack] = (vexRT[Ch2]>driveThreshold  || vexRT[Ch2]< -driveThreshold || vexRT[Ch1]>driveThreshold || vexRT[Ch1]< -driveThreshold) ? -vexRT[Ch2] - vexRT[Ch1] : 0;
	motor[driveLeftFront] = (vexRT[Ch2]>driveThreshold  || vexRT[Ch2]< -driveThreshold || vexRT[Ch1]>driveThreshold || vexRT[Ch1]< -driveThreshold) ? -vexRT[Ch2] - vexRT[Ch1] : 0;
	motor[driveRightBack] = (vexRT[Ch2]>driveThreshold  || vexRT[Ch2]< -driveThreshold || vexRT[Ch1]>driveThreshold || vexRT[Ch1]< -driveThreshold) ? -vexRT[Ch2] + vexRT[Ch1] : 0;
	motor[driveRightFront] = (vexRT[Ch2]>driveThreshold  || vexRT[Ch2]< -driveThreshold || vexRT[Ch1]>driveThreshold || vexRT[Ch1]< -driveThreshold) ? -vexRT[Ch2] + vexRT[Ch1] : 0;
}

static void driverControl(){
	//Drive
	if( vexRT[Btn8R] == 1 ){
		if(!button8RPressed){																//if buttonPressed was previously 0 when Btn8R pressed then
			button8RToggleState = 1 - button8RToggleState;		// change the toggle state
			button8RPressed = 1;															// buttonPressed = 1 so the toggle state doesnt change again
		}
	}
	else button8RPressed = 0;															// the button is not pressed so buttonPressed = 0
		if(button8RToggleState)	invertedDirectionDrive();
	else	normalDrive();

	//Lift

	//XBar

	//Claw

}

#endif
