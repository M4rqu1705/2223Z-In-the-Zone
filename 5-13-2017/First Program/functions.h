#ifndef functions.h
#define functions.h

enum direction{ f, b, l, r};	enum target{ kBase, kArm, kClaw};

typedef struct{
	float baseWidth;	float wheelDiameter;	unsigned short rotationTicks;	//You HAVE to initialize this
	short error; short lastError; short integral; byte output; float Kp; float Ki; float Kd; byte integralLimit;//PID
} robotBase;

typedef struct{
	short maxPotValue;	//You HAVE to initialize this
	short error; short lastError; short integral; byte output; float Kp; float Ki; float Kd; byte integralLimit;//PID
} robotArm;

static robotBase base;  static robotArm arm;

static void initialize(){

	base.baseWidth = 16.5;  base.wheelDiameter = 3.25;  base.rotationTicks = 4000;  arm.maxPotValue = 1024;
	base.Kp = 1.75;  base.Ki = 0.025;  base.Kd=3;  base.integralLimit=20;
	arm.Kp = 1.75;  arm.Ki = 0.025;  arm.Kd=3;  arm.integralLimit=20;
	//Encoders
	SensorValue[LeftEncoder] = 0; SensorValue[RightEncoder] = 0;

	//Gyro
	SensorType[in8] = sensorNone;
	wait1Msec(1000);
	SensorType[in8] = sensorGyro;
	wait1Msec(1500);
}

static void Drive(direction side, unsigned short pulses, byte speed){
	switch(side){
	case f:
		base.error = pulses - ((abs(SensorValue[LeftEncoder]) + abs(SensorValue[RightEncoder]))/2);
		if(base.error != 0 && base.integral < base.integralLimit && base.integral > -base.integralLimit) base.integral += base.error;	else base.integral = 0;
		base.output = (base.Kp * base.error) + (base.Ki * base.integral) + (base.Kd * (base.error != 0 ? base.error - base.lastError : 0));
		base.lastError = base.error;
		motor[port1] = base.output / ((abs(SensorValue[LeftEncoder]) - abs(SensorValue[RightEncoder])) * 0.01 + 1);	//Make sure both sides run at the same rate
		motor[port2] = base.output / ((abs(SensorValue[RightEncoder]) - abs(SensorValue[LeftEncoder])) * 0.01 + 1);	//Make sure both sides run at the same rate
		break;
	case b:
		break;
	case l:
		break;
	case r:
		break;
	}
}
static void Lift(unsigned short ticks, byte speed){

}
static void Intake(unsigned short ticks, byte speed){

}

static void hold(target subs, byte speed){
}
static void stop(target subs){
	switch(subs){
	case kBase:
		break;
	case kArm:
		break;
	case kClaw:
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
