#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#pragma systemfile

typedef struct {
	double KP;
	double KI;
	double KD;
	signed int error;
	signed int lastError;
	double integral;
	unsigned byte integralMax;
	unsigned byte cyclesCounter;
	unsigned byte correctionCycles;
	unsigned byte correctionThreshold;
	unsigned byte timeoutCounter;
	unsigned byte timeout;
	signed byte output;

	bool notDone;
}TEMPLATE_PID;

typedef struct {
	double distanceMultiplier[2];
	double offset;
}TEMPLATE_motionProfile;


enum ENUM_driveDirection{Forward = 0, Backward, PointTurnLeft, PointTurnRight, SwingTurnLeft, SwingTurnRight};
enum ENUM_driveMode{None = 0, PID, MotionProfiling, MotionProfilingAndPID, Gyro };

typedef struct{
	signed byte joystickInputs[2];
	double slewRateOutputs[2];
	signed byte outputs[2];
	TEMPLATE_PID PID;
	TEMPLATE_motionProfile motionProfile;
}TEMPLATE_drive;

typedef struct{
	bool retractButtonPressed;
	bool extendButtonPressed;
	bool intakeState;
	signed byte output;
	TEMPLATE_PID PID;
	TEMPLATE_motionProfile motionProfile;
}TEMPLATE_mobileGoalIntake;

typedef struct{
	signed byte joystickInput;
	signed byte output;
	TEMPLATE_PID PID;
	TEMPLATE_motionProfile motionProfile;
}TEMPLATE_arm;

typedef struct{
	bool pickUpButtonPressed;
	bool depositButtonPressed;
	bool notDone;
	unsigned byte counter;
	signed byte output;
}TEMPLATE_coneIntake;

TEMPLATE_drive drive;
TEMPLATE_mobileGoalIntake mobileGoalIntake;
TEMPLATE_arm arm;
TEMPLATE_coneIntake coneIntake;

#include "math.h"

//Function Prototypes
void initialize();
void resetValues();

void driveOperatorControl(bool simple = false);
void moveDrive(ENUM_driveDirection direction, double pulses, signed byte speed, ENUM_driveMode mode);

void mobileGoalOperatorControl(bool simple = false);
void moveMobileGoal(bool retract, bool loaded = false);

void armOperatorControl(bool simple = false);
void moveArm(unsigned byte position, bool loaded = false);

void coneIntakeOperatorControl();
void moveIntake(bool pickUp, signed byte speed = META_coneIntakeSpeed);

//LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD---//
//#include "fullRobotLCD.h"
//#include "LCD.h"

//Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize //

void initialize(){
#ifdef META_usingLCD
	lcdInit();
	if(bIfiRobotDisabled){
		displayLCDCenteredString(0, "Calibrating GYRO");
	}
#endif
	if(bIfiRobotDisabled){
		SensorType[SENSOR_gyro] = sensorNone;
		wait1Msec(1000);
		SensorType[SENSOR_gyro] = sensorGyro;
		wait1Msec(2000);
		SensorValue[SENSOR_gyro] = 0;
	}

#ifdef META_usingLCD
	do{
		if(bIfiRobotDisabled)	lcdSelect();
		else break;
	}while (!lcdReady);
	//Clear the LCD
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDCenteredString(0, "     2223-Z     ");    //Output 2223-Z on the screen, signaling that the LCD is done
#endif


	motorType[MOTOR_driveLF] = tmotorVex393TurboSpeed_HBridge;
	motorType[MOTOR_driveLM] = tmotorVex393TurboSpeed_HBridge;
	motorType[MOTOR_driveLB] = tmotorVex393TurboSpeed_HBridge;

	motorType[MOTOR_mobileGoalL] = tmotorVex393_HBridge;
	motorType[MOTOR_coneIntake] = tmotorVex393_HBridge;
	motorType[MOTOR_arm] = tmotorVex393_HBridge;
	motorType[MOTOR_mobileGoalR] = tmotorVex393_HBridge;

	motorType[MOTOR_driveRB] = tmotorVex393TurboSpeed_HBridge;
	motorType[MOTOR_driveRM] = tmotorVex393TurboSpeed_HBridge;
	motorType[MOTOR_driveRF] = tmotorVex393TurboSpeed_HBridge;

	bMotorReflected[MOTOR_driveLF] = true;
	bMotorReflected[MOTOR_driveLM] = true;
	bMotorReflected[MOTOR_driveLB] = true;

	bMotorReflected[MOTOR_mobileGoalL] = true;
	bMotorReflected[MOTOR_coneIntake] = true;
	bMotorReflected[MOTOR_arm] = true;
	bMotorReflected[MOTOR_mobileGoalR] = true;

	bMotorReflected[MOTOR_driveRB] = true;
	bMotorReflected[MOTOR_driveRM] = true;
	bMotorReflected[MOTOR_driveRF] = true;

	SensorType[SENSOR_powerExpander] = sensorAnalog;
	SensorType[SENSOR_potMogo] = sensorPotentiometer;
	SensorType[SENSOR_potArm] = sensorPotentiometer;
	SensorType[SENSOR_encoderL] = sensorQuadEncoder;
	SensorType[SENSOR_encoderR] = sensorQuadEncoder;

	resetValues();
	clearDebugStream();
	datalogClear();
}

void resetValues(){

	motor[port1] = motor[port2] = motor[port3] = motor[port4] = motor[port5] = motor[port6] = motor[port7] = motor[port8] = motor[port9] = motor[port10] = 0;

	//Drive Values
	drive.joystickInputs[0] = 0;	drive.joystickInputs[1] = 0;
	drive.slewRateOutputs[0] = 0;	drive.slewRateOutputs[1] = 0;
	drive.outputs[0] = 0; drive.outputs[1] = 0;

	drive.PID.KP = PID_KPdriveUnloaded;
	drive.PID.KI = PID_KIdriveUnloaded;
	drive.PID.KD = PID_KDdriveUnloaded;
	drive.PID.error = 0;
	drive.PID.lastError = 0;
	drive.PID.integral = 0;
	drive.PID.integralMax = PID_integralMaxDrive;
	drive.PID.cyclesCounter = 0;
	drive.PID.correctionCycles = PID_correctionCyclesDriveUnloaded;
	drive.PID.correctionThreshold = PID_correctionThresholdDriveUnloaded;
	drive.PID.timeoutCounter = 0;
	drive.PID.timeout = PID_timeoutDriveUnloaded;
	drive.PID.output = 0;
	drive.PID.notDone = true;

	drive.motionProfile.distanceMultiplier[0] = 1/4;
	drive.motionProfile.distanceMultiplier[1] = 3/4;
	drive.motionProfile.offset = 0;

	//Mobile Goal Values
	mobileGoalIntake.retractButtonPressed = false;
	mobileGoalIntake.extendButtonPressed = false;
	mobileGoalIntake.intakeState = true; //Retracted = 0
	mobileGoalIntake.output = 0;

	mobileGoalIntake.PID.KP = PID_KPmobileGoalIntakeUnloadedRetract;
	mobileGoalIntake.PID.KI = PID_KImobileGoalIntakeUnloadedRetract;
	mobileGoalIntake.PID.KD = PID_KDmobileGoalIntakeUnloadedRetract;
	mobileGoalIntake.PID.error = 0;
	mobileGoalIntake.PID.lastError = 0;
	mobileGoalIntake.PID.integral = 0;
	mobileGoalIntake.PID.integralMax = PID_integralMaxMobileGoalIntake;
	mobileGoalIntake.PID.cyclesCounter = 0;
	mobileGoalIntake.PID.correctionCycles = PID_correctionCyclesMobileGoalIntakeUnloadedRetract;
	mobileGoalIntake.PID.correctionThreshold = PID_correctionThresholdMobileGoalIntakeUnloadedRetract;
	mobileGoalIntake.PID.timeoutCounter = 0;
	mobileGoalIntake.PID.timeout = PID_timeoutMobileGoalIntakeUnloadedRetract;
	mobileGoalIntake.PID.output = 0;
	mobileGoalIntake.PID.notDone = true;

	mobileGoalIntake.motionProfile.distanceMultiplier[0] = 1/5;
	mobileGoalIntake.motionProfile.distanceMultiplier[1] = 4/5;
	mobileGoalIntake.motionProfile.offset = 0;

	//Arm Values
	arm.joystickInput = 0;
	arm.output = 0;

	arm.PID.KP = PID_KParmDown;
	arm.PID.KI = PID_KIarmDown;
	arm.PID.KD = PID_KDarmDown;
	arm.PID.error = 0;
	arm.PID.lastError = 0;
	arm.PID.integral = 0;
	arm.PID.integralMax = PID_integralMaxArm;
	arm.PID.cyclesCounter = 0;
	arm.PID.correctionCycles = PID_correctionCyclesArmDown;
	arm.PID.correctionThreshold = PID_correctionThresholdArmDown;
	arm.PID.timeoutCounter = 0;
	arm.PID.timeout = PID_timeoutArmDown;
	arm.PID.output = 0;
	arm.PID.notDone = true;

	arm.motionProfile.distanceMultiplier[0] = 1/5;
	arm.motionProfile.distanceMultiplier[1] = 4/5;
	arm.motionProfile.offset = 0;

	//Cone Intake Values
	coneIntake.pickUpButtonPressed = false;
	coneIntake.depositButtonPressed = false;
	coneIntake.notDone = true;
	coneIntake.counter = 0;
	coneIntake.output = 0;

}

void driveOperatorControl(bool simple){

	drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
	drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

	if(simple){
		//Calculate output by first doing the operation and then clamping it while converting them to a signed byte
		drive.outputs[0] = (MATH_clamp(-drive.joystickInputs[0] + drive.joystickInputs[1]));
		drive.outputs[1] = (MATH_clamp(-drive.joystickInputs[0] - drive.joystickInputs[1]));

		//Assign calculated output values
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.outputs[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.outputs[1];
	}
	else{

		//Check if joystick values are within a threshold or the will be removed
		if(MATH_withinThreshold(drive.joystickInputs[0], META_driveOpControlThreshold, -META_driveOpControlThreshold)) drive.joystickInputs[0] = 0;

		if(MATH_withinThreshold(drive.joystickInputs[1], META_driveOpControlThreshold, -META_driveOpControlThreshold)) drive.joystickInputs[1] = 0;

		//Apply Slew rate control where the outputs will increase linearly and slowly
		if(drive.slewRateOutputs[0] + META_slewGain < drive.joystickInputs[0] + META_slewGainThreshold) drive.slewRateOutputs[0] += META_slewGain;
		else if(drive.slewRateOutputs[0] - META_slewGain > drive.joystickInputs[0] - META_slewGainThreshold) drive.slewRateOutputs[0] -= META_slewGain;
		else if(drive.joystickInputs[0] == 0  || drive.joystickInputs[1] == 0) drive.slewRateOutputs[0] = 0;


		if(drive.slewRateOutputs[1] + META_slewGain < drive.joystickInputs[1] + META_slewGainThreshold) drive.slewRateOutputs[1] += META_slewGain;
		else if(drive.slewRateOutputs[1] - META_slewGain > drive.joystickInputs[1] - META_slewGainThreshold) drive.slewRateOutputs[1] -= META_slewGain;
		else if(drive.joystickInputs[0] == 0  || drive.joystickInputs[1] == 0) drive.slewRateOutputs[1] = 0;

		drive.slewRateOutputs[0] = MATH_clamp(drive.slewRateOutputs[0]);
		drive.slewRateOutputs[1] = MATH_clamp(drive.slewRateOutputs[1]);

		//Calculate outputs for each side
		drive.outputs[0] = MATH_clamp(MATH_round(-drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
		drive.outputs[1] = MATH_clamp(MATH_round(-drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));

		//Move motors
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.outputs[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.outputs[1];

	}

	/*void moveDrive(ENUM_driveDirection direction, double pulses, signed byte speed, ENUM_driveMode mode){

	}

	void moveDrive(ENUM_driveDirection direction, double turnRadius, double turnDegrees, signed byte speed, ENUM_driveMode mode){

	}*/
}

#endif
