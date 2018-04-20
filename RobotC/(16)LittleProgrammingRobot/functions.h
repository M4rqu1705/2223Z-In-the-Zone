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
	bool notDone;

	STRUCT_driveSide left;	STRUCT_driveSide right;
	STRUCT_SENSOR_lineFollower linefollowers;
}STRUCT_drive;

typedef struct{
	byte joystickInput, output;
	bool notDone;

	STRUCT_slewRate slewRate;
	STRUCT_PID PID;
}STRUCT_lift;

typedef struct{
	byte output;
	bool notDone;

	STRUCT_PID PID;
	STRUCT_slewRate slewRate;
}STRUCT_mobileGoal;

typedef struct{
	byte output;
	int counter;

	STRUCT_slewRate slewRate;
}STRUCT_goliathIntake;

typedef struct{
	byte output;
	bool notDone;
	int counter;

	STRUCT_slewRate slewRate;
	STRUCT_PID PID;
}STRUCT_miniFourbar;

typedef struct{
	int counter;

	STRUCT_goliathIntake goliathIntake;
	STRUCT_miniFourbar miniFourbar;
}STRUCT_coneIntake;

STRUCT_drive drive;
STRUCT_lift lift;
STRUCT_mobileGoal mobileGoal;
STRUCT_coneIntake coneIntake;

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
	motorType[MOTOR_miniFourbarL] = motorType[MOTOR_miniFourbarR] = tmotorVex393TurboSpeed_HBridge;
	motorType[MOTOR_goliathIntake] = tmotorVex393HighSpeed_HBridge;
	motorType[MOTOR_liftL] = motorType[MOTOR_liftR] = tmotorVex393HighSpeed_HBridge;
	motorType[MOTOR_mobileGoal] = tmotorVex393_HBridge;

	//#define MOTOR_liftType tmotorVex393HighSpeed_HBridge
	//#define MOTOR_miniFourbarType tmotorVex393_HBridge

	//Invert necessary motors
	bMotorReflected[port1] = MOTOR_inverted[0];
	bMotorReflected[port2] = MOTOR_inverted[1];
	bMotorReflected[port3] = MOTOR_inverted[2];
	bMotorReflected[port4] = MOTOR_inverted[3];
	bMotorReflected[port5] = MOTOR_inverted[4];
	bMotorReflected[port6] = MOTOR_inverted[5];
	bMotorReflected[port7] = MOTOR_inverted[6];
	bMotorReflected[port8] = MOTOR_inverted[7];
	bMotorReflected[port9] = MOTOR_inverted[8];
	bMotorReflected[port10] = MOTOR_inverted[9];



	//Identify types of sensors
	SensorType[SENSOR_encoderL] = SensorType[SENSOR_encoderR] = sensorQuadEncoder;
	SensorType[SENSOR_lineTrackerLeft] = SensorType[SENSOR_lineTrackerRight] =  sensorLineFollower;
	SensorType[SENSOR_potentiometerLift] = sensorPotentiometer;

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
	motor[port1] = motor[port2] = motor[port3] = motor[port4] = motor[port5] = motor[port6] = motor[port7] = motor[port8] = motor[port9] = motor[port10] = 0;

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

	slewRateInit(drive.left.slewRate, 0, 0, 1, 5, META_slewRateMaxSlope);
	slewRateInit(drive.right.slewRate, 0, 0, 1, 5, META_slewRateMaxSlope);
	slewRateInit(lift.slewRate, 0, 0, 1, 5, META_slewRateMaxSlope);
	slewRateInit(mobileGoal.slewRate, 0, 0, 1, 5, META_slewRateMaxSlope);
	slewRateInit(coneIntake.goliathIntake.slewRate, 0, 0, 1, 5, META_slewRateMaxSlope);
	slewRateInit(coneIntake.miniFourbar.slewRate, 0, 0, 1, 5, META_slewRateMaxSlope);
	pidInit(drive.left.PID, 0, 0, 0, 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
	pidInit(drive.right.PID, 0, 0, 0, 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
	pidInit(lift.PID, 0, 0, 0, 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
	pidInit(mobileGoal.PID, 0, 0, 0, 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
	pidInit(coneIntake.miniFourbar.PID, 0, 0, 0, 0, 0, 0, 0, 0, PID_correctionCyclesDrive);
}

void operatorControl(){
	//----------------------------------------Drive----------------------------------------

	//Assign joystick values to joystickInputs array
	drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
	drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

	//Make sure that the joystickInput variables are outside the range of the constant JOYSTICK_driveThreshold
	if(fabs(drive.joystickInputs[0]) < JOYSTICK_driveThreshold)	drive.joystickInputs[0] = 0;
	if(fabs(drive.joystickInputs[1]) < JOYSTICK_driveThreshold)	drive.joystickInputs[1] = 0;

	//Calculate arcade drive outputs and clamp them
	drive.left.output = clamp(drive.joystickInputs[0] + drive.joystickInputs[1]);
	drive.right.output = clamp(drive.joystickInputs[0] - drive.joystickInputs[1]);

	//Update the slewRate struct values with new, recently calculated values. This will serve to operate the slewRateControl function later in the program
	slewRateInit(drive.left.slewRate, motor[MOTOR_driveLF], drive.left.output, 1, 5, META_slewRateMaxSlope);
	slewRateInit(drive.right.slewRate, motor[MOTOR_driveRF], drive.right.output, 1, 5, META_slewRateMaxSlope);

	//---------------------------------------- Lift ----------------------------------------

	//Assign joystick value to joystickInput variable
	lift.joystickInput = vexRT[JOYSTICK_lift];

	//Make sure that the joystickInput variable is outside the range of the constant JOYSTICK_liftThreshold
	if(fabs(lift.joystickInput) < JOYSTICK_liftThreshold)	lift.joystickInput = 0;

	//Update the lift output variable
	lift.output = lift.joystickInput;

	//Update the slewRate struct values with new, recently calculated values. This will serve to operate the slewRateControl function later in the program
	slewRateInit(lift.slewRate, motor[MOTOR_liftR], lift.output, 1, 5, META_slewRateMaxSlope);

	//---------------------------------------- Mobile Goal Intake ----------------------------------------

	//If mobileGoalExtend button is pressed, update the output variable accordingly
	if(vexRT[JOYSTICK_mobileGoalE])	mobileGoal.output = 127;
	//If mobileGoalRetract button is pressed, update the output variable accordingly
	else if(vexRT[JOYSTICK_mobileGoalR])	mobileGoal.output = -127;
	//If no button is pressed, do not move the mobile goal intake
	else	mobileGoal.output = 0;

	//Update the slewRate struct values with new, recently calculated values. This will serve to operate the slewRateControl function later in the program
	slewRateInit(mobileGoal.slewRate, motor[MOTOR_mobileGoal], mobileGoal.output, 1, 5, META_slewRateMaxSlope);

	//---------------------------------------- Cone intake ----------------------------------------

	//If coneUp button is pressed, raise the mini fourbar after picking up cone and once ready deposit it
	if(vexRT[JOYSTICK_coneU]){
		//Increase coneIntake.counter by one. Equals counter = counter + 1
		coneIntake.counter++;
		//If less than 13 cycles have passed
		if(coneIntake.counter < 13){
			//Pick up cone
			coneIntake.goliathIntake.output = 127;
			//Keep the miniFourbar down
			coneIntake.miniFourbar.output = clamp(round(calculatePID(coneIntake.miniFourbar.PID, META_miniFourbarDown, SensorValue[SENSOR_potentiometerMiniFourbar])));
		}
		else{
			//pidInit(coneIntake.miniFourbar.PID,
			//Rise miniFoubar
			coneIntake.miniFourbar.output = clamp(round(calculatePID(coneIntake.miniFourbar.PID, META_miniFourbarUp, SensorValue[SENSOR_potentiometerMiniFourbar])));
			//Keep intaking cone in order for it not to fall
			coneIntake.goliathIntake.output = 50;

			//Check if miniFoubar is done and is already in position
			if(coneIntake.miniFourbar.notDone){
				//Check if the miniFoubar output is within a specified threshold
				if(fabs(coneIntake.miniFourbar.PID.output) < coneIntake.miniFourbar.PID.correctionThreshold){
					//Add to a counter, making sure to take into account time that miniFoubar has been within the threshold
					coneIntake.miniFourbar.counter++;
					//If enough time has passed, mechanism is done
					if(5 <= coneIntake.miniFourbar.counter)	coneIntake.miniFourbar.notDone = false;
					//If not enough time has passed, mechanism is not done
					else	coneIntake.miniFourbar.notDone = true;
				}
				else coneIntake.miniFourbar.notDone = true;
			}
			//If mechanism is done
			else{
				//Just to keep the miniFoubar in the same position
				coneIntake.miniFourbar.output = clamp(round(calculatePID(coneIntake.miniFourbar.PID, META_miniFourbarUp, SensorValue[SENSOR_potentiometerMiniFourbar])));
				//Deposit cone
				coneIntake.goliathIntake.output = -127;
			}
		}
	}
	else if(vexRT[JOYSTICK_coneD]){
		//pidInit(coneIntake.miniFourbar.PID,
		//Move and keep miniFourbar down
		coneIntake.miniFourbar.output = clamp(round(calculatePID(coneIntake.miniFourbar.PID, META_miniFourbarDown, SensorValue[SENSOR_potentiometerMiniFourbar])));
		//Do not move the loliath intake on the way down
		coneIntake.goliathIntake.output = 0;
	}
	else{
		//Reset counter
		coneIntake.counter = 0;
		//Do not move miniFourbar nor goliath intake
		coneIntake.miniFourbar.output = coneIntake.goliathIntake.output = 0;
	}

	//---------------------------------------- Goliath intake ----------------------------------------
	if(vexRT[JOYSTICK_goliathIntakeI])	coneIntake.goliathIntake.output = 127;
	else if(vexRT[JOYSTICK_goliathIntakeD])	coneIntake.goliathIntake.output = -127;
	else coneIntake.goliathIntake.output = 0;

	//---------------------------------------- Mini Fourbar ----------------------------------------
	if(vexRT[JOYSTICK_miniFourbarU])	coneIntake.miniFourbar.output = 127;
	else if(vexRT[JOYSTICK_miniFourbarD])	coneIntake.miniFourbar.output = -127;
	else coneIntake.miniFourbar.output = 0;


	slewRateInit(coneIntake.miniFourbar.slewRate, motor[MOTOR_miniFourbarL], coneIntake.miniFourbar.output, 1, 5, META_slewRateMaxSlope);
	slewRateInit(coneIntake.goliathIntake.slewRate, motor[MOTOR_goliathIntake], coneIntake.goliathIntake.output, 1, 5, META_slewRateMaxSlope);

	// nerd <3

	//Update all motors using slewrate control
	slewRateControl(MOTOR_driveLF, drive.left.slewRate);
	slewRateControl(MOTOR_driveLB, drive.left.slewRate);
	slewRateControl(MOTOR_driveRF, drive.right.slewRate);
	slewRateControl(MOTOR_driveRB, drive.right.slewRate);
	slewRateControl(MOTOR_liftL, lift.slewRate);
	slewRateControl(MOTOR_liftR, lift.slewRate);
	slewRateControl(MOTOR_mobileGoal, mobileGoal.slewRate);
	slewRateControl(MOTOR_miniFourbarL, coneIntake.miniFourbar.slewRate);
	slewRateControl(MOTOR_miniFourbarR, coneIntake.miniFourbar.slewRate);
	slewRateControl(MOTOR_goliathIntake, coneIntake.goliathIntake.slewRate);
}

#endif
