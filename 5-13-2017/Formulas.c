struct robotBase{
	//Encoder Values
	float baseWidth;				/*Robot's base's width*/
	float wheelDiameter;			/*Robot's wheels' diameter?*/
	unsigned short pulses;			/*How many pulses does the robot needs to move?*/
	//Gyro values
	unsigned short rotationTicks;	/*How many ticks the gyro reads so the robot rotates once completely?*/
	unsigned short ticks;			/*How many ticks does the robot needs to rotate*/	
	
	short degrees;					/*How many degrees the person wants the base to rotate*/
} base;

struct robotArm{
	short maxPotValue;				/*Potentiometer's highest value*/
	short ticks;					/*To what position does the arm need to rotate?*/
	
	short degrees;					/*Degrees the person wants the arm to rotate*/
	short position;					/*Current arm position*/
} arm;

#define base.baseWidth 16.5
#define base.wheelDiameter 3.25
#define base.rotationTicks 4000

#define arm.maxPotValue 1024

void main(){
//Pulses needed to rotate the base a total of 'degrees' degrees with Encoders
	SensorValue[LeftEncoder] = 0; SensorValue[RightEncoder] = 0;
	
	base.degrees = 90;
	base.pulses = FLOOR( ((base.degrees/360) * base.baseWidth * 3.1415926535897932384626433832795) / (base.wheelDiameter / 360) );

//Pulses needed to rotate the base a total of 'degrees' degrees with Gyroscope
	base.ticks = CEIL(base.degrees*(base.rotationTicks/360));

//Pulses needed to rotate the arm a total of 'degrees' degrees with Potentiometers
	arm.degrees = -90;
	arm.ticks = arm.position + (FLOOR(arm.degrees * (arm.maxPotValue / 270)))
}