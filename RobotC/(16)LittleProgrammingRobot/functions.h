#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#pragma systemfile

typedef struct{
	byte presentCurrent, desiredCurrent;
	short presentCycle, desiredCycle;
	float maxSlope;
}STRUCT_slewRate;

typedef struct {
	float KP, KI, KD, integral, derivative;
	int error, lastError;
	byte output, correctionThreshold;
}STRUCT_PID;

typedef struct{
	tSensors port;
	bool inverted;
	int currentPosition, previousPosition, pulsesPerRevolution;
	float RPM;
}STRUCT_SENSOR_encoder;

typedef struct{
	tSensors port[2];
	int sensedCounter[2];
	int averageDistance;
}STRUCT_SENSOR_lineFollower;

typedef struct{
	byte output;
	int doneCounter;
	STRUCT_slewRate slewRate;	
	STRUCT_PID PID; 
	STRUCT_SENSOR_encoder encoder;
}STRUCT_driveSide;

typedef struct{
	byte joystickInputs[2];
	int previousGyro;	bool rectify;

	STRUCT_driveSide left;	STRUCT_driveSide right;
	STRCUT_SENSOR_lineFollower linefollowers;
}STRUCT_drive;

typedef struct{
	byte joystickInput, output;

	STRUCT_slewRate slewRate;
	STRUCT_PID PID;
}STRUCT_lift;

typedef struct{
	byte buttonInputs[2];

	STRUCT_PID PID;
}

STRUCT_drive drive;
STRUCT_lift lift;

#include "utils.h"

//Function prototypes
void resetValues();
void initialize();

bool moveDrive();
bool rotateDrive();



void initialize(){
	//Initiate motors with their types
	motorType[MOTOR_driveLF] = motorType[MOTOR_driveLB] =  tmotorVex393TurboSpeed_HBridge;
	motorType[MOTOR_driveRB] = motorType[MOTOR_driveRF] = tmotorVex393TurboSpeed_HBridge;

	//#define MOTOR_liftType tmotorVex393HighSpeed_HBridge
	//#define MOTOR_miniFourbarType tmotorVex393_HBridge

	//Invert necessary motors
	bMotorReflected[MOTOR_driveLF] = false;
	bMotorReflected[MOTOR_driveLB] = true;
	bMotorReflected[MOTOR_driveRB] = false;
	bMotorReflected[MOTOR_driveRF] = true;

	//Identify types of sensors
	SensorType[SENSOR_encoderL] = SensorType[SENSOR_encoderR] = sensorQuadEncoder;
	SensorType[SENSOR_accelerometer] = sensorAnalog;
	SensorType[SENSOR_lineTrackerLeft] = SensorType[SENSOR_lineTrackerRight] =  sensorLineFollower;

	//Clear out variables
	resetValues();
	//Clear out debug stream
	clearDebugStream();
	//Clear out datalog
	datalogClear();

	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	//Calibrate Gyro
	if(bIfiRobotDisabled){
		SensorType[SENSOR_gyro] = sensorNone;
		wait1Msec(1000);
		SensorType[SENSOR_gyro] = sensorGyro;
		wait1Msec(1500);
		SensorValue[SENSOR_gyro] = 0;

		SensorScale[SENSOR_gyro] = META_gyroCalibrationConstant;
		SensorFullCount[SENSOR_gyro] = 3600;
	}


}

void resetValues(){
	motor[port1] = motor[port2] = motor[port9] = motor[port10] = 0;

	drive.left.output = drive.right.output = 0;
	drive.left.encoder.port = SENSOR_encoderL;
	drive.right.encoder.port = SENSOR_encoderR;
	drive.left.encoder.inverted = META_encoderLInverted;
	drive.right.encoder.inverted = META_encoderRInverted;
	drive.left.encoder.currentPosition = drive.left.encoder.previousPosition = 0;
	drive.right.encoder.currentPosition = drive.right.encoder.previousPosition = 0;
	drive.left.encoder.pulsesPerRevolution = drive.right.encoder.pulsesPerRevolution = META_drivePulsesPerRevolution;
	drive.left.encoder.RPM = drive.right.encoder.RPM = 0;

	drive.left.doneCounter = drive.right.doneCounter = 0;

	drive.joystickInputs[0] = drive.joystickInputs[1] = 0;
	drive.previousGyro = 0;
	drive.rectify = false;

	drive.linefollowers.port[0] = SENSOR_lineTrackerLeft;
	drive.linefollowers.port[1] = SENSOR_lineTrackerRight;
	drive.linefollowers.sensedCounter[0] = drive.linefollowers.sensedCounter[1] = 0;
	drive.linefollowers.averageDistance = 0;

	slewRateInit(drive.left.slewRate, 0, 0, 0, 0, META_slewRateMaxSlope);
	slewRateInit(drive.right.slewRate, 0, 0, 0, 0, META_slewRateMaxSlope);
	slewRateInit(lift.slewRate, 0, 0, 0, 0, META_slewRateMaxSlope);
	pidInit(drive.left.PID, 0, 0, 0, 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
	pidInit(drive.right.PID, 0, 0, 0, 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
}

void operatorControl(int Cycle){
	//Drive
	drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
	drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

	if(checkWithinThreshold(drive.joystickInputs[0], JOYSTICK_driveThreshold, -JOYSTICK_driveThreshold))	drive.joystickInputs[0] = 0;
	if(checkWithinThreshold(drive.joystickInputs[1], JOYSTICK_driveThreshold, -JOYSTICK_driveThreshold))	drive.joystickInputs[1] = 0;

	drive.left.output = clamp(drive.joystickInputs[0] + drive.joystickInputs[1]);
	drive.right.output = clamp(drive.joystickInputs[0] - drive.joystickInputs[1]);

	slewRateInit(drive.left.slewRate, motor[MOTOR_driveLF], drive.left.output, 1, 5, META_slewRateMaxSlope);
	slewRateInit(drive.right.slewRate, motor[MOTOR_driveRF], drive.right.output, 1, 5, META_slewRateMaxSlope);

	//Lift
	lift.joystickInput = vexRT[JOYSTICK_lift];
	if(checkWithinThreshold(lift.joystick, JOYSTICK_liftThreshold, -JOYSTICK_liftThreshold)) lift.joystickInput = 0;

	lift.output = lift.joystickInput;
	slewRateInit(lift.slewRate, motor[MOTOR_liftL], lift.output, 1, 5, META_slewRateMaxSlope);
	slewRateInit(lift.slewRate, motor[MOTOR_liftR], lift.output, 1, 5, META_slewRateMaxSlope);

	//Mobile Goal intake
	if(vexRT[])


	//Update all motors using slewrate control
	slewRateControl(MOTOR_driveLF, drive.left.slewRate);
	slewRateControl(MOTOR_driveLB, drive.left.slewRate);
	slewRateControl(MOTOR_driveRF, drive.right.slewRate);
	slewRateControl(MOTOR_driveRB, drive.right.slewRate);
	slewRateControl(MOTOR_liftL, lift.slewRate);
	slewRateControl(MOTOR_liftR, lift.slewRate);

	l
}

#endif