#ifndef functions.h
#define functions.h

enum direction{ f, b, l, r};	enum target{ kBase, kArm, kXBar, kClaw};	enum sensorTarget{ kEncoderLeft, kEncoderRight, kPotentiometerArm, kPotentiometerXBar};

typedef struct{ float kP; float kI; float kD; short error; short lastError; short integral; byte integralLimit; byte output;} PID;	//Variables for PID
typedef struct{ float width;	float wheelDiameter;	unsigned short rotationTicks;} baseValues;	//You HAVE to initialize this
typedef struct{ unsigned short maxPotValue;} armValues;	//You HAVE to initialize this
typedef struct{ float kg; float estimate; float previousEstimate; float errorEstimate; float previousEstimate; float previousErrorEstimate; byte errorMeasurement;} kalmanFilter;

static baseValues baseConstant;  static armValues armConstant; static PID drive; static PID arm;
static kalmanFilter encoderLeft; static kalmanFilter encoderRight; static kalmanFilter potentiometerArm; static kalmanFilter potentiometerXBar;

static bool baseDone = false, armDone = false, clawDone = false;

static void initialize(){
	baseConstant.width	= 16.5;
	baseConstant.wheelDiameter = 3.25;
	baseConstant.rotationTicks= 4000;																						//How many ticks does the gyro read on ONE full rotation
	armConstant.maxPotValue = 1024;																							//Since some potentiomenters read up to 4096, this is used just in case
	drive.kP = 1.27; drive.kI = 0.025; drive.kD = 3; drive.integralLimit = 20;	//Define the PID constants for base
	arm.kP = 1.75;  arm.kI = 0.025;  arm.kD = 3;  arm.integralLimit = 20;				//Define the PID constants for arm
	SensorValue[LeftEncoder] = 0; SensorValue[RightEncoder] = 0;								//Reset encoder values
	//Gyro
	SensorType[in8] = sensorNone;
	wait1Msec(1000);
	SensorType[in8] = sensorGyro;
	wait1Msec(1500);
	encoderLeft.kg = 0; encoderLeft.estimate=0; encoderLeft.previousEstimate = 0; encoderLeft.errorEstimate = 0; encoderLeft.previousEstimate = 0; encoderLeft.previousErrorEstimate = 0; encoderLeft.errorMeasurement = 5;
	encoderRight.kg = 0; encoderRight.estimate=0; encoderRight.previousEstimate = 0; encoderRight.errorEstimate = 0; encoderRight.previousEstimate = 0; encoderRight.previousErrorEstimate = 0; encoderRight.errorMeasurement = 5;
	potentiometerArm.kg = 0; potentiometerArm.estimate=0; potentiometerArm.previousEstimate = 0; potentiometerArm.errorEstimate = 0; potentiometerArm.previousEstimate = 0; potentiometerArm.previousErrorEstimate = 0; potentiometerArm.errorMeasurement = 5;
	potentiometerXBar.kg = 0; potentiometerXBar.estimate=0; potentiometerXBar.previousEstimate = 0; potentiometerXBar.errorEstimate = 0; potentiometerXBar.previousEstimate = 0; potentiometerXBar.previousErrorEstimate = 0; potentiometerXBar.errorMeasurement = 5;
}

static void resetValues(){
	SensorValue[LeftEncoder] = 0; SensorValue[RightEncoder] = 0;																					//Reset encoder values
	encoderLeft.kg = 0; encoderLeft.estimate=0; encoderLeft.previousEstimate = 0; encoderLeft.errorEstimate = 0; encoderLeft.previousEstimate = 0; encoderLeft.previousErrorEstimate = 0; encoderLeft.errorMeasurement = 5;
	encoderRight.kg = 0; encoderRight.estimate=0; encoderRight.previousEstimate = 0; encoderRight.errorEstimate = 0; encoderRight.previousEstimate = 0; encoderRight.previousErrorEstimate = 0; encoderRight.errorMeasurement = 5;
	potentiometerArm.kg = 0; potentiometerArm.estimate=0; potentiometerArm.previousEstimate = 0; potentiometerArm.errorEstimate = 0; potentiometerArm.previousEstimate = 0; potentiometerArm.previousErrorEstimate = 0; potentiometerArm.errorMeasurement = 5;
	potentiometerXBar.kg = 0; potentiometerXBar.estimate=0; potentiometerXBar.previousEstimate = 0; potentiometerXBar.errorEstimate = 0; potentiometerXBar.previousEstimate = 0; potentiometerXBar.previousErrorEstimate = 0; potentiometerXBar.errorMeasurement = 5;
	drive.error = 0; drive.lastError = 0; drive.integral = 0; drive.integralLimit = 0; drive.output = 0;	//Reset PID variables for base
	arm.error = 0; arm.lastError = 0; arm.integral = 0; arm.integralLimit = 0; arm.output = 0;						//Reset PID variables for arm
	baseDone = false; armDone = false; clawDone = false;
}

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
		motor[liftLeft] = speed;
		break;
	case kClaw:
		motor[clawMotor] = speed;
		break;
	}
}

static void stop(target subs){
	switch(subs){
	case kBase:
		motor[driveLeftBack] = 0;
		motor[driveLeftFront] = 0;
		motor[driveRightBack] = 0;
		motor[driveRightFront] = 0;
		break;
	case kArm:
		motor[liftLeft] = 0;
		motor[liftRight] = 0;
		break;
	case kClaw:
		motor[clawMotor] = 0;
		break;
	}
}

static int getSensor(sensorTarget sensor){
	switch(sensor){
	case kEncoderLeft:
		encoderLeft.kg = encoderLeft.errorEstimate / (encoderLeft.errorEstimate - encoderLeft.errorMeasurement);
		encoderLeft.estimate = encoderLeft.previousEstimate + encoderLeft.kg *(SensorValue[LeftEncoder]-encoderLeft.previousEstimate);
		encoderLeft.errorEstimate = (1 - encoderLeft.kg) * encoderLeft.previousErrorEstimate;
		encoderLeft.previousEstimate = encoderLeft.estimate; encoderLeft.previousErrorEstimate = encoderLeft.errorEstimate;
		return(encoderLeft.estimate);
	case kEncoderRight:
		encoderRight.kg = encoderRight.errorEstimate / (encoderRight.errorEstimate - encoderRight.errorMeasurement);
		encoderRight.estimate = encoderRight.previousEstimate + encoderRight.kg *(SensorValue[RightEncoder]-encoderRight.previousEstimate);
		encoderRight.errorEstimate = (1 - encoderRight.kg) * encoderRight.previousErrorEstimate;
		encoderRight.previousEstimate = encoderRight.estimate; encoderRight.previousErrorEstimate = encoderRight.errorEstimate;
		return(encoderRight.estimate);
	case kPotentiometerArm:
		potentiometerArm.kg = potentiometerArm.errorEstimate / (potentiometerArm.errorEstimate - potentiometerArm.errorMeasurement);
		potentiometerArm.estimate = potentiometerArm.previousEstimate + potentiometerArm.kg *(SensorValue[armPot]-potentiometerArm.previousEstimate);
		potentiometerArm.errorEstimate = (1 - potentiometerArm.kg) * potentiometerArm.previousErrorEstimate;
		potentiometerArm.previousEstimate = potentiometerArm.estimate; potentiometerArm.previousErrorEstimate = potentiometerArm.errorEstimate;
		return(potentiometerArm.estimate);
	case kPotentiometerXBar:
		potentiometerXBar.kg = potentiometerXBar.errorEstimate / (potentiometerXBar.errorEstimate - potentiometerXBar.errorMeasurement);
		potentiometerXBar.estimate = potentiometerXBar.previousEstimate + potentiometerXBar.kg *(SensorValue[xBarPot]-potentiometerXBar.previousEstimate);
		potentiometerXBar.errorEstimate = (1 - potentiometerXBar.kg) * potentiometerXBar.previousErrorEstimate;
		potentiometerXBar.previousEstimate = potentiometerXBar.estimate; potentiometerXBar.previousErrorEstimate = potentiometerXBar.errorEstimate;
		return(potentiometerXBar.estimate);
	}
	return(0);
}

static void move(direction side, unsigned short pulses, byte speed, bool useGyro = true){
	if(side == l || side == r){
		if(useGyro){
			pulses = ceil(pulses *(baseConstant.rotationTicks/360));
		}
		else{
			pulses = floor(((pulses/360) * baseConstant.width * 3.1415926535897932384626433832795)/(baseConstant.wheelDiameter / 360));
		}
	}
	//PID
	drive.error = pulses - ((abs(getSensor(kEncoderLeft)) + abs(getSensor(kEncoderRight)))/2);
	if(drive.error != 0 && drive.integral < drive.integralLimit && drive.integral > -drive.integralLimit) drive.integral += drive.error;	else drive.integral = 0;
	drive.output = (drive.kP * drive.error) + (drive.kI * drive.integral) + (drive.kD * (drive.error != 0 ? drive.error - drive.lastError : 0));
	drive.lastError = drive.error;

	//Move motors based on PID values and "rectify" robot if necessary
	switch(side){
	case f:
		motor[driveLeftFront] = drive.output / ((abs(getSensor(kEncoderLeft)) - abs(getSensor(kEncoderRight))) * 0.01 + 1);
		motor[driveLeftBack] = drive.output / ((abs(getSensor(kEncoderLeft)) - abs(getSensor(kEncoderRight))) * 0.01 + 1);
		motor[driveRightFront] = drive.output / ((abs(getSensor(kEncoderRight)) - abs(getSensor(kEncoderLeft))) * 0.01 + 1);
		motor[driveRightBack] = drive.output / ((abs(getSensor(kEncoderRight)) - abs(getSensor(kEncoderLeft))) * 0.01 + 1);
		break;
	case b:
		motor[driveLeftFront] = -drive.output / ((abs(getSensor(kEncoderLeft)) - abs(getSensor(kEncoderRight))) * 0.01 + 1);
		motor[driveLeftBack] = -drive.output / ((abs(getSensor(kEncoderLeft)) - abs(getSensor(kEncoderRight))) * 0.01 + 1);
		motor[driveRightFront] = -drive.output / ((abs(getSensor(kEncoderRight)) - abs(getSensor(kEncoderLeft))) * 0.01 + 1);
		motor[driveRightBack] = -drive.output / ((abs(getSensor(kEncoderRight)) - abs(getSensor(kEncoderLeft))) * 0.01 + 1);
		break;
	case l:
		motor[driveLeftFront] = -drive.output / ((abs(getSensor(kEncoderLeft)) - abs(getSensor(kEncoderRight))) * 0.01 + 1);
		motor[driveLeftBack] = -drive.output / ((abs(getSensor(kEncoderLeft)) - abs(getSensor(kEncoderRight))) * 0.01 + 1);
		motor[driveRightFront] = drive.output / ((abs(getSensor(kEncoderRight)) - abs(getSensor(kEncoderLeft))) * 0.01 + 1);
		motor[driveRightBack] = drive.output / ((abs(getSensor(kEncoderRight)) - abs(getSensor(kEncoderLeft))) * 0.01 + 1);
		break;
	case r:
		motor[driveLeftFront] = drive.output / ((abs(getSensor(kEncoderLeft)) - abs(getSensor(kEncoderRight))) * 0.01 + 1);
		motor[driveLeftBack] = drive.output / ((abs(getSensor(kEncoderLeft)) - abs(getSensor(kEncoderRight))) * 0.01 + 1);
		motor[driveRightFront] = -drive.output / ((abs(getSensor(kEncoderRight)) - abs(getSensor(kEncoderLeft))) * 0.01 + 1);
		motor[driveRightBack] = -drive.output / ((abs(getSensor(kEncoderRight)) - abs(getSensor(kEncoderLeft))) * 0.01 + 1);
		break;
	}
	if(getSensor(kEncoderLeft) < pulses+10 && getSensor(kEncoderLeft) > pulses-10) baseDone = true;
}
static void lift(unsigned short ticks, byte speed){
	if(SensorValue[armPot] < ticks+10 && SensorValue[LeftEncoder] > ticks-10) armDone = true;
}
static void intake(unsigned short ticks, byte speed){
}

/*
baseLeft.grados = 90;
baseLeft.pulses = floor( ((baseLeft.grados/360) * baseLeft.baseWidth * 3.1415926535897932384626433832795) / (baseLeft.wheelDiameter / 360) );

//Pulses needed to rotate the base a total of 'degrees' degrees with Gyroscope
baseLeft.ticks = ceil(baseLeft.grados *(baseLeft.rotationTicks/360));

//Pulses needed to rotate the arm a total of 'degrees' degrees with Potentiometers
armLeft.grados = -90;
armLeft.ticks = armLeft.position + (floor(armLeft.grados * (armLeft.maxPotValue / 270)));
*/
#endif
