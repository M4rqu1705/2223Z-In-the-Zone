#ifndef functions.h
#define functions.h

enum direction{ f, b, l, r};	enum target{ kBase, kArm, kClaw};

typedef struct{ float kP; float kI; float kD; short error; short lastError; short integral; short integralLimit; byte output;} PID;

typedef struct{ float width;	float wheelDiameter;	unsigned short rotationTicks;} baseValues;	//You HAVE to initialize this

typedef struct{ short maxPotValue;} armValues;	//You HAVE to initialize this

static baseValues baseConstant;  static armValues armConstant; static PID drive; static PID arm;

static void initialize(){
	baseConstant.width	= 16.5;
	baseConstant.wheelDiameter = 3.25;
	baseConstant.rotationTicks= 4000;																						//How many ticks does the gyro read on ONE full rotation
	armConstant.maxPotValue = 1024;																							//Since some potentiomenters read up to 4096, this is used just in case
	drive.kP = 1.75; drive.kI = 0.025; drive.kD = 3; drive.integralLimit = 20;	//Define the PID constants for base
	arm.kP = 1.75;  arm.kI = 0.025;  arm.kD = 3;  arm.integralLimit = 20;				//Define the PID constants for arm
	SensorValue[LeftEncoder] = 0; SensorValue[RightEncoder] = 0;								//Reset encoder values
	//Gyro
	SensorType[in8] = sensorNone;
	wait1Msec(1000);
	SensorType[in8] = sensorGyro;
	wait1Msec(1500);
}

static void resetValues(){
	SensorValue[LeftEncoder] = 0; SensorValue[RightEncoder] = 0;																					//Reset encoder values
	drive.error = 0; drive.lastError = 0; drive.integral = 0; drive.integralLimit = 0; drive.output = 0;	//Reset PID variables for base
	arm.error = 0; arm.lastError = 0; arm.integral = 0; arm.integralLimit = 0; arm.output = 0;						//Reset PID variables for arm
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
	drive.error = pulses - ((abs(SensorValue[LeftEncoder]) + abs(SensorValue[RightEncoder]))/2);
	if(drive.error != 0 && drive.integral < drive.integralLimit && drive.integral > -drive.integralLimit) drive.integral += drive.error;	else drive.integral = 0;
	drive.output = (drive.kP * drive.error) + (drive.kI * drive.integral) + (drive.kD * (drive.error != 0 ? drive.error - drive.lastError : 0));
	drive.lastError = drive.error;

	switch(side){
	case f:
		//Move motors based on PID values and "rectify" robot if necessary
		motor[baseLeftFront] = drive.output / ((abs(SensorValue[LeftEncoder]) - abs(SensorValue[RightEncoder])) * 0.01 + 1);
		motor[baseLeftBack] = drive.output / ((abs(SensorValue[LeftEncoder]) - abs(SensorValue[RightEncoder])) * 0.01 + 1);
		motor[baseRightFront] = -(drive.output / ((abs(SensorValue[RightEncoder]) - abs(SensorValue[LeftEncoder])) * 0.01 + 1));
		motor[baseRightBack] = -(drive.output / ((abs(SensorValue[RightEncoder]) - abs(SensorValue[LeftEncoder])) * 0.01 + 1));
		break;
	case b:
		//Move motors based on PID values and "rectify" robot if necessary
		motor[baseLeftFront] = -(drive.output / ((abs(SensorValue[LeftEncoder]) - abs(SensorValue[RightEncoder])) * 0.01 + 1));
		motor[baseLeftBack] = -(drive.output / ((abs(SensorValue[LeftEncoder]) - abs(SensorValue[RightEncoder])) * 0.01 + 1));
		motor[baseRightFront] = drive.output / ((abs(SensorValue[RightEncoder]) - abs(SensorValue[LeftEncoder])) * 0.01 + 1);
		motor[baseRightBack] = drive.output / ((abs(SensorValue[RightEncoder]) - abs(SensorValue[LeftEncoder])) * 0.01 + 1);
		break;
	case l:
		//Move motors based on PID values and "rectify" robot if necessary
		motor[baseLeftFront] = -drive.output / ((abs(SensorValue[LeftEncoder]) - abs(SensorValue[RightEncoder])) * 0.01 + 1);
		motor[baseLeftBack] = -drive.output / ((abs(SensorValue[LeftEncoder]) - abs(SensorValue[RightEncoder])) * 0.01 + 1);
		motor[baseRightFront] = -drive.output / ((abs(SensorValue[RightEncoder]) - abs(SensorValue[LeftEncoder])) * 0.01 + 1);
		motor[baseRightBack] = -drive.output / ((abs(SensorValue[RightEncoder]) - abs(SensorValue[LeftEncoder])) * 0.01 + 1);
		break;
	case r:
		//Move motors based on PID values and "rectify" robot if necessary
		motor[baseLeftFront] = drive.output / ((abs(SensorValue[LeftEncoder]) - abs(SensorValue[RightEncoder])) * 0.01 + 1);
		motor[baseLeftBack] = drive.output / ((abs(SensorValue[LeftEncoder]) - abs(SensorValue[RightEncoder])) * 0.01 + 1);
		motor[baseRightFront] = drive.output / ((abs(SensorValue[RightEncoder]) - abs(SensorValue[LeftEncoder])) * 0.01 + 1);
		motor[baseRightBack] = drive.output / ((abs(SensorValue[RightEncoder]) - abs(SensorValue[LeftEncoder])) * 0.01 + 1);
		break;
	}
}
static void lift(unsigned short ticks, byte speed){

}
static void intake(unsigned short ticks, byte speed){

}

static void hold(target subs, byte speed){
	switch(subs){
	case kBase:
		motor[baseLeftBack] = speed;
		motor[baseLeftFront] = speed;
		motor[baseRightBack] = -speed;
		motor[baseRightFront] = -speed;
		break;
	case kArm:
		motor[armLeft] = speed;
		motor[armRight] = -speed;
		break;
	case kClaw:
		motor[clawMotor] = speed;
		break;
	}
}
static void stop(target subs){
	switch(subs){
	case kBase:
		motor[baseLeftBack] = 0;
		motor[baseLeftFront] = 0;
		motor[baseRightBack] = 0;
		motor[baseRightFront] = 0;
		break;
	case kArm:
		motor[armLeft] = 0;
		motor[armRight] = 0;
		break;
	case kClaw:
		motor[clawMotor] = 0;
		break;
	}
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
