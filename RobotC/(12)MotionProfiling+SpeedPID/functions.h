#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#pragma systemfile

typedef struct {
	float KP;
	float KI;
	float KD;
	int error;
	int lastError;
	float integral;
	ubyte integralMax;
	ubyte cyclesCounter;
	ubyte correctionCycles;
	ubyte correctionThreshold;
	ubyte timeoutCounter;
	ubyte timeout;
	byte output;

	bool notDone;
}TEMPLATE_PID;

typedef struct {
	float distanceMultiplier[2];
	float offset;
	int previousPosition;
}TEMPLATE_motionProfile;

enum ENUM_driveMode{None = 0, PID, Acceleration, Gyro };

typedef struct{
	byte joystickInputs[2];
	float slewRateOutputs[2];
	byte outputs[2];
	TEMPLATE_PID PID;
	TEMPLATE_motionProfile motionProfile;
}TEMPLATE_drive;

typedef struct{
	byte joystickInput;
	bool retractButtonPressed;
	bool extendButtonPressed;
	bool intakeState;
	byte output;
	TEMPLATE_PID PID;
	TEMPLATE_motionProfile motionProfile;
}TEMPLATE_mobileGoalIntake;

typedef struct{
	byte joystickInput;
	byte output;
	TEMPLATE_PID PID;
	TEMPLATE_motionProfile motionProfile;
}TEMPLATE_arm;

typedef struct{
	bool notDone;
	ubyte counter;
	byte output;
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
void driveForward(ENUM_driveMode mode, float pulses, byte speed);
void driveBackwards(ENUM_driveMode mode, float pulses, byte speed);
void turnRight(ENUM_driveMode mode, float pulses, float turnRadius, byte speed);
void turnLeft(ENUM_driveMode mode, float pulses, float turnRadius, byte speed);

void mobileGoalOperatorControl(bool notAnalog = false);
void moveMobileGoal(bool retract);

void armOperatorControl(bool analog = true);
void moveArm(ENUM_driveMode mode, ubyte position);

void coneIntakeOperatorControl();
void moveConeIntake(bool pickUp, ubyte cycles);

//LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD---//
#include "LCDcalibrate.h"
//#include "LCD.h"

//Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize //

void initialize(){
#ifdef META_usingLCD
	LCD_init();
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
		SensorScale[SENSOR_gyro] = 136;
		SensorFullCount[SENSOR_gyro] = 18000;
	}

#ifdef META_usingLCD
	LCD_calibrate();
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

	bMotorReflected[MOTOR_driveLF] = false;
	bMotorReflected[MOTOR_driveLM] = false;
	bMotorReflected[MOTOR_driveLB] = true;

	bMotorReflected[MOTOR_mobileGoalL] = true;
	bMotorReflected[MOTOR_coneIntake] = true;
	bMotorReflected[MOTOR_arm] = true;
	bMotorReflected[MOTOR_mobileGoalR] = false;

	bMotorReflected[MOTOR_driveRB] = false;
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

	drive.motionProfile.distanceMultiplier[0] = 1;
	drive.motionProfile.distanceMultiplier[1] = 1.1;	//When greater than 1, it is not used
	drive.motionProfile.offset = 25;
	drive.motionProfile.previousPosition = 0;

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
	mobileGoalIntake.motionProfile.previousPosition = 0;

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
	arm.motionProfile.previousPosition = 0;

	//Cone Intake Values
	coneIntake.notDone = true;
	coneIntake.counter = 0;
	coneIntake.output = 0;

	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	SensorValue[SENSOR_gyro] = 0;

}

void armLoaded(bool loaded){
	if(loaded){
		arm.PID.KP = PID_KParmUp;
		arm.PID.KI = PID_KIarmUp;
		arm.PID.KD = PID_KDarmUp;
		arm.PID.integralMax = PID_integralMaxArm;
		arm.PID.correctionCycles = PID_correctionCyclesArmUp;
		arm.PID.correctionThreshold = PID_correctionThresholdArmUp;
		arm.PID.timeout = PID_timeoutArmUp;
	}
	else{
		arm.PID.KP = PID_KParmDown;
		arm.PID.KI = PID_KIarmDown;
		arm.PID.KD = PID_KDarmDown;
		arm.PID.correctionCycles = PID_correctionCyclesArmDown;
		arm.PID.correctionThreshold = PID_correctionThresholdArmDown;
		arm.PID.timeout = PID_timeoutArmDown;
	}
}

void loadMobileGoal(bool loaded, bool retract, bool usingGyro){
	if(loaded){

		if(usingGyro){
			drive.PID.KP = PID_KPdriveGyroLoaded;
			drive.PID.KI = PID_KIdriveGyroLoaded;
			drive.PID.KD = PID_KDdriveGyroLoaded;
		}
		else{
			drive.PID.KP = PID_KPdriveLoaded;
			drive.PID.KI = PID_KIdriveLoaded;
			drive.PID.KD = PID_KDdriveLoaded;
		}
		drive.PID.integralMax = PID_integralMaxDrive;
		drive.PID.correctionCycles = PID_correctionCyclesDriveLoaded;
		drive.PID.correctionThreshold = PID_correctionThresholdDriveLoaded;
		drive.PID.timeout = PID_timeoutDriveLoaded;

		if(retract){
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntakeLoadedRetract;
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntakeLoadedRetract;
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntakeLoadedRetract;
			mobileGoalIntake.PID.integralMax = PID_integralMaxMobileGoalIntake;
			mobileGoalIntake.PID.correctionCycles = PID_correctionCyclesMobileGoalIntakeLoadedRetract;
			mobileGoalIntake.PID.correctionThreshold = PID_correctionThresholdMobileGoalIntakeLoadedRetract;
			mobileGoalIntake.PID.timeout = PID_timeoutMobileGoalIntakeLoadedRetract;
		}
		else{
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntakeLoadedExtend;
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntakeLoadedExtend;
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntakeLoadedExtend;
			mobileGoalIntake.PID.integralMax = PID_integralMaxMobileGoalIntake;
			mobileGoalIntake.PID.correctionCycles = PID_correctionCyclesMobileGoalIntakeLoadedExtend;
			mobileGoalIntake.PID.correctionThreshold = PID_correctionThresholdMobileGoalIntakeLoadedExtend;
			mobileGoalIntake.PID.timeout = PID_timeoutMobileGoalIntakeLoadedExtend;
		}
	}
	else{
		if(usingGyro){
			drive.PID.KP = PID_KPdriveGyroUnloaded;
			drive.PID.KI = PID_KIdriveGyroUnloaded;
			drive.PID.KD = PID_KDdriveGyroUnloaded;
		}
		else{
			drive.PID.KP = PID_KPdriveUnloaded;
			drive.PID.KI = PID_KIdriveUnloaded;
			drive.PID.KD = PID_KDdriveUnloaded;
		}

		drive.PID.integralMax = PID_integralMaxDrive;
		drive.PID.correctionCycles = PID_correctionCyclesDriveUnloaded;
		drive.PID.correctionThreshold = PID_correctionThresholdDriveUnloaded;
		drive.PID.timeout = PID_timeoutDriveUnloaded;

		if(retract){
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntakeUnloadedRetract;
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntakeUnloadedRetract;
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntakeUnloadedRetract;
			mobileGoalIntake.PID.integralMax = PID_integralMaxMobileGoalIntake;
			mobileGoalIntake.PID.correctionCycles = PID_correctionCyclesMobileGoalIntakeUnloadedRetract;
			mobileGoalIntake.PID.correctionThreshold = PID_correctionThresholdMobileGoalIntakeUnloadedRetract;
			mobileGoalIntake.PID.timeout = PID_timeoutMobileGoalIntakeUnloadedRetract;
		}
		else{
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntakeUnloadedExtend;
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntakeUnloadedExtend;
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntakeUnloadedExtend;
			mobileGoalIntake.PID.integralMax = PID_integralMaxMobileGoalIntake;
			mobileGoalIntake.PID.correctionCycles = PID_correctionCyclesMobileGoalIntakeUnloadedExtend;
			mobileGoalIntake.PID.correctionThreshold = PID_correctionThresholdMobileGoalIntakeUnloadedExtend;
			mobileGoalIntake.PID.timeout = PID_timeoutMobileGoalIntakeUnloadedExtend;
		}
	}
}

//Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive --//
void driveOperatorControl(bool simple){

	drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
	drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

	if(simple){
		//Calculate output by first doing the operation and then clamping it while converting them to a byte
		drive.outputs[0] = (MATH_clamp(drive.joystickInputs[0] + drive.joystickInputs[1]));
		drive.outputs[1] = (MATH_clamp(drive.joystickInputs[0] - drive.joystickInputs[1]));

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
		else if(drive.joystickInputs[0] == 0) drive.slewRateOutputs[0] = 0;


		if(drive.slewRateOutputs[1] + META_slewGain < drive.joystickInputs[1] + META_slewGainThreshold) drive.slewRateOutputs[1] += META_slewGain;
		else if(drive.slewRateOutputs[1] - META_slewGain > drive.joystickInputs[1] - META_slewGainThreshold) drive.slewRateOutputs[1] -= META_slewGain;
		else if(drive.joystickInputs[1] == 0) drive.slewRateOutputs[1] = 0;

		drive.slewRateOutputs[0] = MATH_clamp(drive.slewRateOutputs[0]);
		drive.slewRateOutputs[1] = MATH_clamp(drive.slewRateOutputs[1]);

		//Calculate outputs for each side
		drive.outputs[0] = MATH_clamp(MATH_round(drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
		drive.outputs[1] = MATH_clamp(MATH_round(drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));

		//Move motors
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.outputs[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.outputs[1];

	}
}

void driveForward(ENUM_driveMode mode, float pulses, byte speed){
	pulses = MATH_inchesToPulses(pulses);

	switch(mode){
	case PID:
		MATH_calculatePID(drive.PID, pulses, ((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2));
		drive.outputs[0] = drive.outputs[1] = MATH_map(drive.PID.output, 127, -127, speed, -speed);
		break;
	case Acceleration:
		if(((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2) < pulses){
			drive.outputs[0] = drive.outputs[1] = MATH_motionProfile(drive.motionProfile, ((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2), pulses, speed);
		}
		else{
			drive.outputs[0] = drive.outputs[1] = 0;
			drive.PID.notDone = false;
		}
		break;
	default:
		if(((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2) < pulses){
			drive.outputs[0] = drive.outputs[1] = speed;
		}
		else{
			drive.outputs[0] = drive.outputs[1] = 0;
			drive.PID.notDone = false;
		}
	}

	//writeDebugStream("Converted Pulses=%f\t", pulses);	writeDebugStreamLine("Sensors = %f, Outputs = %d",((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2) ,drive.outputs[0]);

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.outputs[0];
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.outputs[1];

}

void driveBackwards(ENUM_driveMode mode, float pulses, byte speed){
	pulses = MATH_inchesToPulses(pulses);

	switch(mode){
	case PID:
		MATH_calculatePID(drive.PID, pulses, ((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2));
		drive.outputs[0] = drive.outputs[1] = MATH_map(drive.PID.output, 127, -127, speed, -speed);
		break;
	case Acceleration:
		if(((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2) < pulses){
			drive.outputs[0] = drive.outputs[1] = MATH_motionProfile(drive.motionProfile, ((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2), pulses, speed);
		}
		else{
			drive.outputs[0] = drive.outputs[1] = 0;
			drive.PID.notDone = false;
		}
		break;
	default:
		if(((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2) < pulses){
			drive.outputs[0] = drive.outputs[1] = speed;
		}
		else{
			drive.outputs[0] = drive.outputs[1] = 0;
			drive.PID.notDone = false;
		}
	}

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = -drive.outputs[0];
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = -drive.outputs[1];
}

void turnRight(ENUM_driveMode mode, float pulses, float turnRadius, byte speed){
	if(mode == Gyro){
		pulses = MATH_degreesToTicks(pulses);
	}
	else{
		pulses = MATH_degreesToPulses(pulses, turnRadius);
	}

	switch(mode){
	case PID:
		MATH_calculatePID(drive.PID, pulses, abs(SensorValue[SENSOR_encoderL]));
		drive.outputs[0] = MATH_map(drive.PID.output, 127, -127, speed, -speed);
		MATH_calculatePID(drive.PID, MATH_swingTurnInside(turnRadius, pulses), abs(SensorValue[SENSOR_encoderR]));
		drive.outputs[1] = -MATH_map(drive.PID.output, 127, -127, MATH_swingTurnInside(turnRadius, drive.outputs[0]), -MATH_swingTurnInside(turnRadius, drive.outputs[0]));
		if(pulses < 0){
			drive.outputs[0] = -drive.outputs[0];
			drive.outputs[1] = -drive.outputs[1];
		}
		break;
	case Gyro:
		writeDebugStreamLine("Turning %f degrees with gyro", pulses);
		MATH_calculatePID(drive.PID, pulses, abs(SensorValue[SENSOR_gyro]));
		drive.outputs[0] = MATH_map(drive.PID.output, 127, -127, speed, -speed);
		drive.outputs[1] = -MATH_map(drive.PID.output, 127, -127, speed, -speed);
		break;
	default:
		if(abs(SensorValue[SENSOR_encoderL]) < pulses){
			drive.outputs[0] = speed;
			drive.outputs[1] = MATH_swingTurnInside(turnRadius, speed);
			if(pulses < 0){
				drive.outputs[0] = -drive.outputs[0];
				drive.outputs[1] = -drive.outputs[1];
			}
		}
		else{
			drive.outputs[0] = drive.outputs[1] = 0;
			drive.PID.notDone = false;
		}
	}
	writeDebugStream("Converted Pulses=%f\t", pulses);	writeDebugStreamLine("Sensor L = %f, Outputs = %d",abs(SensorValue[SENSOR_encoderL]),drive.outputs[0]);

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.outputs[0];
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.outputs[1];
}

void turnLeft(ENUM_driveMode mode, float pulses, float turnRadius, byte speed){
	if(mode == Gyro){
		pulses = MATH_degreesToTicks(pulses);
	}
	else{
		pulses = MATH_degreesToPulses(pulses, turnRadius);
	}

	switch(mode){
	case PID:
		MATH_calculatePID(drive.PID, pulses, abs(SensorValue[SENSOR_encoderR]));
		drive.outputs[1] = MATH_map(drive.PID.output, 127, -127, speed, -speed);
		MATH_calculatePID(drive.PID, MATH_swingTurnInside(turnRadius, pulses), abs(SensorValue[SENSOR_encoderL]));
		drive.outputs[0] = -MATH_map(drive.PID.output, 127, -127, MATH_swingTurnInside(turnRadius, drive.outputs[1]), -MATH_swingTurnInside(turnRadius, drive.outputs[1]));
		if(pulses < 0){
			drive.outputs[0] = -drive.outputs[0];
			drive.outputs[1] = -drive.outputs[1];
		}
		break;
	case Gyro:
		writeDebugStreamLine("Turning %f degrees with gyro", pulses);
		MATH_calculatePID(drive.PID, pulses, abs(SensorValue[SENSOR_gyro]));
		drive.outputs[0] = -MATH_map(drive.PID.output, 127, -127, speed, -speed);
		drive.outputs[1] = MATH_map(drive.PID.output, 127, -127, speed, -speed);
		break;
	default:
		if(abs(SensorValue[SENSOR_encoderL]) < pulses){
			drive.outputs[0] = speed;
			drive.outputs[1] = MATH_swingTurnInside(turnRadius, speed);
			if(pulses < 0){
				drive.outputs[0] = -drive.outputs[0];
				drive.outputs[1] = -drive.outputs[1];
			}
		}
		else{
			drive.outputs[0] = drive.outputs[1] = 0;
			drive.PID.notDone = false;
			if(pulses < 0){
				drive.outputs[0] = -drive.outputs[0];
				drive.outputs[1] = -drive.outputs[1];
			}
		}
	}
	writeDebugStream("Converted Pulses=%f\t", pulses);	writeDebugStreamLine("Sensor R = %f, Outputs = %d",abs(SensorValue[SENSOR_encoderR]),drive.outputs[1]);

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.outputs[0];
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.outputs[1];
}


//--Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake//
void moveMobileGoal(bool retract){
	if(retract){
		MATH_calculatePID(mobileGoalIntake.PID, META_mogoRetracted, SensorValue[SENSOR_potMogo]);
		motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = -mobileGoalIntake.PID.output;
	}
	else{
		MATH_calculatePID(mobileGoalIntake.PID, META_mogoExtended, SensorValue[SENSOR_potMogo]);
		motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = -mobileGoalIntake.PID.output;
	}
}

void mobileGoalOperatorControl(bool notAnalog){
	if(notAnalog){
		if(vexRT[JOYSTICK_mobileGoalE]){
			motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = META_mogoMaxOutput;
		}
		else if(vexRT[JOYSTICK_mobileGoalR]){
			motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = -META_mogoMaxOutput;
		}
		else{
			motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = 0;
		}
	}
	else{
		mobileGoalIntake.joystickInput = vexRT[JOYSTICK_arm];

		if(MATH_withinThreshold(mobileGoalIntake.joystickInput, META_mogoOpControlThreshold, -META_mogoOpControlThreshold)){
			mobileGoalIntake.joystickInput = 0;
		}

		if((SensorValue[SENSOR_potMogo] >= META_mogoExtended) && (SensorValue[SENSOR_potMogo] <= META_mogoRetracted)){
			motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = mobileGoalIntake.joystickInput;
		}
		else if((SensorValue[SENSOR_potMogo] <= META_mogoExtended) && (mobileGoalIntake.joystickInput < 0)){
			motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = mobileGoalIntake.joystickInput;
		}
		else if((SensorValue[SENSOR_potMogo] >= META_mogoRetracted) && (mobileGoalIntake.joystickInput > 0)){
			motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = mobileGoalIntake.joystickInput;
		}
		else{
			motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = 0;
		}
	}
}

//Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm -- Arm//
void moveArm(ENUM_driveMode mode, ubyte position){
	if(mode == None){
		switch(position){
		case 0:
			if(SensorValue[SENSOR_potArm] > META_armDown) arm.output = META_armMaxOutput;
			else{
				arm.output = 0;
				arm.PID.notDone = false;
			}
			break;
		case 1:
			if(SensorValue[SENSOR_potArm] < META_armDown) arm.output = -META_armMaxOutput;
			else{
				arm.output = 0;
				arm.PID.notDone = false;
			}
			break;
		case 2:
			if(SensorValue[SENSOR_potArm] < META_armScore && !MATH_withinThreshold(SensorValue[SENSOR_potArm] , META_armScore + 15, META_armScore - 15)) arm.output = -META_armMaxOutput;
			else if(SensorValue[SENSOR_potArm] > META_armScore && !MATH_withinThreshold(SensorValue[SENSOR_potArm] , META_armScore + 15, META_armScore - 15)) arm.output = META_armMaxOutput;
			else {
				arm.output = 0;
				arm.PID.notDone = false;
			}
			break;
		case 3:
			if(SensorValue[SENSOR_potArm] < META_armLoader && !MATH_withinThreshold(SensorValue[SENSOR_potArm] , META_armLoader + 15, META_armLoader - 15)) arm.output = -META_armMaxOutput;
			else if(SensorValue[SENSOR_potArm] > META_armLoader && !MATH_withinThreshold(SensorValue[SENSOR_potArm] , META_armLoader + 15, META_armLoader - 15)) arm.output = META_armMaxOutput;
			else{
				arm.output = 0;
				arm.PID.notDone = false;
			}
			break;
		default:
		}
	}
	else if(mode == PID){
		switch(position){
		case 0:
			MATH_calculatePID(arm.PID, META_armDown, SensorValue[SENSOR_potArm]);
			break;
		case 1:
			MATH_calculatePID(arm.PID, META_armUp, SensorValue[SENSOR_potArm]);
			break;
		case 2:
			MATH_calculatePID(arm.PID, META_armScore, SensorValue[SENSOR_potArm]);
			break;
		case 3:
			MATH_calculatePID(arm.PID, META_armLoader, SensorValue[SENSOR_potArm]);
			break;
		default:
		}
		arm.output = -arm.PID.output;
	}

	motor[MOTOR_arm] = arm.output;
}

void armOperatorControl(bool analog){
	if(analog){
		motor[MOTOR_arm] = vexRT[JOYSTICK_arm];
	}
	else{
		if(vexRT[JOYSTICK_mobileGoalE]){
			motor[MOTOR_arm] = META_armMaxOutput;
		}
		else if(vexRT[JOYSTICK_mobileGoalR]){
			motor[MOTOR_arm] = -META_armMaxOutput;
		}
		else{
			motor[MOTOR_arm] = 0;
		}
	}
}

// Cone Intake --  Cone Intake --  Cone Intake --  Cone Intake --  Cone Intake --  Cone Intake --  Cone Intake --  Cone Intake//
void moveConeIntake(bool pickUp, ubyte cycles){
	if(pickUp){
		motor[MOTOR_coneIntake] = -META_coneIntakeSpeed;
	}
	else{
		motor[MOTOR_coneIntake] = META_coneIntakeSpeed;
	}
	coneIntake.counter++;

	if(coneIntake.counter >= cycles){
		coneIntake.notDone = false;
		motor[MOTOR_coneIntake] = 0;
	}
}

void coneIntakeOperatorControl(){
	if(vexRT[JOYSTICK_coneIntakeD]){
		motor[MOTOR_coneIntake] = META_coneIntakeMaxOutput;
	}
	else if(vexRT[JOYSTICK_coneIntakeP]){
		motor[MOTOR_coneIntake] = -META_coneIntakeMaxOutput;
	}
	else{
		motor[MOTOR_coneIntake] = 0;
	}
}

#endif
