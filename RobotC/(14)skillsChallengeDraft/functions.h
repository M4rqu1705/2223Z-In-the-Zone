/*   functions.h - Necessary functions controlling robot motion             *
*    Copyright (C) <2018>  Marcos Ricardo Pesante Colón                     *
*                                                                           *
*    This program is free software: you can redistribute it and/or modify   *
*    it under the terms of the GNU General Public License as published by   *
*    the Free Software Foundation, either version 3 of the License, or      *
*    (at your option) any later version.                                    *
*                                                                           *
*    This program is distributed in the hope that it will be useful,        *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of         *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
*    GNU General Public License for more details.                           *
*                                                                           *
*    You should have received a copy of the GNU General Public License      *
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

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

	float previousPosition;
	byte output;
	bool notDone;
}TEMPLATE_PID;

typedef struct {
	float distanceMultiplier[2];
	float offsets[2];

}TEMPLATE_motionProfile;

enum ENUM_driveMode{None = 0, PID, Acceleration, Gyro };

typedef struct{
	byte output;
	float previousPosition;

	TEMPLATE_PID PID;
	TEMPLATE_motionProfile motionProfile;
}TEMPLATE_driveSide;

typedef struct{
	byte joystickInputs[2];
	float slewRateOutputs[2];
	TEMPLATE_driveSide left;
	TEMPLATE_driveSide right;
}TEMPLATE_drive;

typedef struct{
	byte joystickInput;
	bool retractButtonPressed;
	bool extendButtonPressed;
	byte output;
	TEMPLATE_PID PID;
}TEMPLATE_mobileGoalIntake;

typedef struct{
	byte joystickInput;
	byte output;
	TEMPLATE_PID PID;
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

void LOADED_arm(bool loaded);
void LOADED_mobileGoal(bool loaded, bool retract, bool usingGyro);

void DRIVE_operatorControl(bool simple = false);
void DRIVE_forward(ENUM_driveMode mode, float pulses, float speed);
void DRIVE_backwards(ENUM_driveMode mode, float pulses, byte speed);
void DRIVE_turnRight(ENUM_driveMode mode, float pulses, float turnRadius, byte speed);
void DRIVE_turnLeft(ENUM_driveMode mode, float pulses, float turnRadius, byte speed);

void MOBILEGOAL_retract(bool retract = true);
void MOBILEGOAL_operatorControl(bool notAnalog = false);

void ARM_move(ENUM_driveMode mode, ubyte position);
void ARM_operatorControl(bool analog = false);

void GOLIATH_pickUp(bool pickUp = true, ubyte cycles);
void GOLIATH_operatorControl();


//LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD----LCD---//
#include "LCDcalibrate.h"
//#include "LCD.h"

//Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize -- Initialize //

void initialize(){
	/*
	if(bIfiRobotDisabled){
		SensorType[SENSOR_gyro] = sensorNone;
		wait1Msec(1000);
		SensorType[SENSOR_gyro] = sensorGyro;
		wait1Msec(2000);
		SensorValue[SENSOR_gyro] = 0;
		SensorScale[SENSOR_gyro] = 136;
		//SensorScale[SENSOR_gyro] = 141;
		SensorFullCount[SENSOR_gyro] = 3600;
	}*/

	motorType[MOTOR_driveLF] = motorType[MOTOR_driveLM] = motorType[MOTOR_driveLB] =  tmotorVex393TurboSpeed_HBridge;
	motorType[MOTOR_mobileGoalL] = motorType[MOTOR_mobileGoalR] = tmotorVex393_HBridge;
	motorType[MOTOR_coneIntake] = motorType[MOTOR_arm] = tmotorVex393_HBridge;
	motorType[MOTOR_driveRB] = motorType[MOTOR_driveRM] = motorType[MOTOR_driveRF] = tmotorVex393TurboSpeed_HBridge;

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
	SensorType[SENSOR_potMogo] = SensorType[SENSOR_potArm] = sensorPotentiometer;
	SensorType[SENSOR_encoderL] = SensorType[SENSOR_encoderR] = sensorQuadEncoder;

	resetValues();
	clearDebugStream();
	datalogClear();
}

void resetValues(){

	motor[port1] = motor[port2] = motor[port3] = motor[port4] = motor[port5] = motor[port6] = motor[port7] = motor[port8] = motor[port9] = motor[port10] = 0;

	//Drive Values
	drive.joystickInputs[0] = drive.joystickInputs[1] = 0;
	drive.slewRateOutputs[0] = drive.slewRateOutputs[1] = 0;
	drive.left.output = drive.right.output = 0;
	drive.left.previousPosition = drive.right.previousPosition = 0;

	drive.left.PID.error = 0;
	drive.left.PID.lastError = 0;
	drive.left.PID.integral = 0;
	drive.left.PID.integralMax = PID_integralMaxDrive;
	drive.left.PID.cyclesCounter = 0;
	drive.left.PID.output = 0;
	drive.left.PID.notDone = true;

	drive.right.PID.error = 0;
	drive.right.PID.lastError = 0;
	drive.right.PID.integral = 0;
	drive.right.PID.integralMax = PID_integralMaxDrive;
	drive.right.PID.cyclesCounter = 0;
	drive.right.PID.output = 0;
	drive.right.PID.notDone = true;

	drive.left.motionProfile.distanceMultiplier[0] = 0.1;
	drive.left.motionProfile.distanceMultiplier[1] = 0.8;
	drive.left.motionProfile.offsets[0] = 25;
	drive.left.motionProfile.offsets[1] = 15;

	drive.right.motionProfile.distanceMultiplier[0] = 0.1;
	drive.right.motionProfile.distanceMultiplier[1] = 0.8;
	drive.right.motionProfile.offsets[0] = 25;
	drive.right.motionProfile.offsets[1] = 15;

	//Mobile Goal Values
	mobileGoalIntake.retractButtonPressed = mobileGoalIntake.extendButtonPressed = false;
	mobileGoalIntake.output = 0;
	mobileGoalIntake.joystickInput = 0;

	mobileGoalIntake.PID.error = 0;
	mobileGoalIntake.PID.lastError = 0;
	mobileGoalIntake.PID.integral = 0;
	mobileGoalIntake.PID.integralMax = PID_integralMaxMobileGoalIntake;
	mobileGoalIntake.PID.cyclesCounter = 0;
	mobileGoalIntake.PID.output = 0;
	mobileGoalIntake.PID.notDone = true;

	//Arm Values
	arm.joystickInput = 0;
	arm.output = 0;

	arm.PID.error = 0;
	arm.PID.lastError = 0;
	arm.PID.integral = 0;
	arm.PID.integralMax = PID_integralMaxArm;
	arm.PID.cyclesCounter = 0;
	arm.PID.output = 0;
	arm.PID.notDone = true;

	//Cone Intake Values
	coneIntake.notDone = true;
	coneIntake.counter = 0;
	coneIntake.output = 0;

	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	SensorValue[SENSOR_gyro] = 0;
}

void LOADED_arm(bool loaded){
	if(loaded){
		arm.PID.KP = PID_KParmUp;
		arm.PID.KI = PID_KIarmUp;
		arm.PID.KD = PID_KDarmUp;
		arm.PID.correctionCycles = PID_correctionCyclesArmUp;
		arm.PID.correctionThreshold = PID_correctionThresholdArmUp;
	}
	else{
		arm.PID.KP = PID_KParmDown;
		arm.PID.KI = PID_KIarmDown;
		arm.PID.KD = PID_KDarmDown;
		arm.PID.correctionCycles = PID_correctionCyclesArmDown;
		arm.PID.correctionThreshold = PID_correctionThresholdArmDown;
	}
}

void LOADED_mobileGoal(bool loaded, bool retract, bool usingGyro){
	if(loaded){
		if(usingGyro){
			drive.left.PID.KP = drive.right.PID.KP = PID_KPdriveGyroLoaded;
			drive.left.PID.KI = drive.right.PID.KI = PID_KIdriveGyroLoaded;
			drive.left.PID.KD = drive.right.PID.KD = PID_KDdriveGyroLoaded;
		}
		else{
			drive.left.PID.KP = drive.right.PID.KP = PID_KPdriveLoaded;
			drive.left.PID.KI = drive.right.PID.KI = PID_KIdriveLoaded;
			drive.left.PID.KD = drive.right.PID.KD = PID_KDdriveLoaded;
		}
		drive.left.PID.correctionCycles = drive.right.PID.correctionCycles = PID_correctionCyclesDriveLoaded;
		drive.left.PID.correctionThreshold = drive.right.PID.correctionThreshold = PID_correctionThresholdDriveLoaded;

		if(retract){
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntakeLoadedRetract;
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntakeLoadedRetract;
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntakeLoadedRetract;
			mobileGoalIntake.PID.correctionCycles = PID_correctionCyclesMobileGoalIntakeLoadedRetract;
			mobileGoalIntake.PID.correctionThreshold = PID_correctionThresholdMobileGoalIntakeLoadedRetract;
		}
		else{
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntakeLoadedExtend;
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntakeLoadedExtend;
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntakeLoadedExtend;
			mobileGoalIntake.PID.correctionCycles = PID_correctionCyclesMobileGoalIntakeLoadedExtend;
			mobileGoalIntake.PID.correctionThreshold = PID_correctionThresholdMobileGoalIntakeLoadedExtend;
		}
	}

	else{
		if(usingGyro){
			drive.left.PID.KP = drive.right.PID.KP = PID_KPdriveGyroUnloaded;
			drive.left.PID.KI = drive.right.PID.KI = PID_KIdriveGyroUnloaded;
			drive.left.PID.KD = drive.right.PID.KD = PID_KDdriveGyroUnloaded;
		}
		else{
			drive.left.PID.KP = drive.right.PID.KP = PID_KPdriveUnloaded;
			drive.left.PID.KI = drive.right.PID.KI = PID_KIdriveUnloaded;
			drive.left.PID.KD = drive.right.PID.KD = PID_KDdriveUnloaded;
		}
		drive.left.PID.correctionCycles = drive.right.PID.correctionCycles = PID_correctionCyclesDriveUnloaded;
		drive.left.PID.correctionThreshold = drive.right.PID.correctionThreshold = PID_correctionThresholdDriveUnloaded;

		if(retract){
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntakeUnloadedRetract;
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntakeUnloadedRetract;
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntakeUnloadedRetract;
			mobileGoalIntake.PID.correctionCycles = PID_correctionCyclesMobileGoalIntakeUnloadedRetract;
			mobileGoalIntake.PID.correctionThreshold = PID_correctionThresholdMobileGoalIntakeUnloadedRetract;
		}
	}
}

//Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive --//
void DRIVE_operatorControl(bool simple){

	drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
	drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

	if(simple){
		//Calculate output by first doing the operation and then clamping it while converting them to a byte
		drive.left.output = (MATH_clamp(drive.joystickInputs[0] + drive.joystickInputs[1]));
		drive.right.output = (MATH_clamp(drive.joystickInputs[0] - drive.joystickInputs[1]));

		//Assign calculated output values
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.left.output;
		motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.right.output;
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
		drive.left.output = MATH_clamp(MATH_round(drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
		drive.right.output = MATH_clamp(MATH_round(drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));

		//Move motors
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.left.output;
		motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.right.output;

	}
}

void DRIVE_forward(ENUM_driveMode mode, float pulses, float speed){
	pulses = MATH_inchesToPulses(pulses);

	switch(mode){
	case PID:
		speed = MATH_map(speed, 127, 0, 240, 0);
		MATH_calculatePID(drive.left.PID, MATH_motionProfile(drive.left.motionProfile, abs(SensorValue[SENSOR_encoderL]), pulses, speed), MATH_getSpeed(drive.left.previousPosition, abs(SensorValue[SENSOR_encoderL])));
		MATH_calculatePID(drive.right.PID, MATH_motionProfile(drive.right.motionProfile, abs(SensorValue[SENSOR_encoderR]), pulses, speed), MATH_getSpeed(drive.right.previousPosition, abs(SensorValue[SENSOR_encoderR])));

		drive.left.output = drive.left.PID.output;
		drive.right.output = drive.right.PID.output;
		break;

	default:
		if(((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2) < pulses){
			drive.left.output = drive.right.output = speed;
		}
		else{
			drive.left.output = drive.right.output = 0;
			drive.left.PID.notDone = drive.right.PID.notDone = false;
		}
	}


	//Move motors
	drive.left.output += (abs(SensorValue[SENSOR_encoderR]) - abs(SensorValue[SENSOR_encoderL]));

	datalogAddValue(0, drive.left.output);
	datalogAddValue(1, drive.right.output);
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.left.output;
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.right.output;

}

void DRIVE_backwards(ENUM_driveMode mode, float pulses, byte speed){
	pulses = MATH_inchesToPulses(pulses);

	switch(mode){
	case PID:
		speed = MATH_map(speed, 127, 0, 240, 0);
		MATH_calculatePID(drive.left.PID, MATH_motionProfile(drive.left.motionProfile, abs(SensorValue[SENSOR_encoderL]), pulses, speed), MATH_getSpeed(drive.left.previousPosition, abs(SensorValue[SENSOR_encoderL])));
		MATH_calculatePID(drive.right.PID, MATH_motionProfile(drive.right.motionProfile, abs(SensorValue[SENSOR_encoderR]), pulses, speed), MATH_getSpeed(drive.right.previousPosition, abs(SensorValue[SENSOR_encoderR])));

		drive.left.output = drive.left.PID.output;
		drive.right.output = drive.right.PID.output;
		break;

	default:
		if(((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2) < pulses){
			drive.left.output = drive.right.output = -speed;
		}
		else{
			drive.left.output = drive.right.output = 0;
			drive.left.PID.notDone = drive.right.PID.notDone = false;
		}
	}


	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = -drive.left.output;
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = -drive.right.output;

}

void DRIVE_turnRight(ENUM_driveMode mode, float pulses, float turnRadius, byte speed){
	if(mode == Gyro){
		pulses = MATH_degreesToTicks(pulses);
	}
	else{
		pulses = MATH_degreesToPulses(pulses, turnRadius);
	}

	switch(mode){
	case PID:
		speed = MATH_map(speed, 127, 0, 240, 0);

		MATH_calculatePID(drive.left.PID, MATH_motionProfile(drive.left.motionProfile, abs(SensorValue[SENSOR_encoderL]), fabs(pulses), speed), MATH_getSpeed(drive.left.previousPosition, abs(SensorValue[SENSOR_encoderL])));

		MATH_calculatePID(drive.right.PID, MATH_swingTurnInside(turnRadius, MATH_motionProfile(drive.right.motionProfile, abs(SensorValue[SENSOR_encoderR]), fabs(pulses), speed)), MATH_getSpeed(drive.right.previousPosition, abs(SensorValue[SENSOR_encoderR])));

		if(pulses >= 0){
			drive.left.output = drive.left.PID.output;
			drive.right.output = drive.right.PID.output;
		}
		else if(pulses < 0){
			drive.left.output = -drive.left.PID.output;
			drive.right.output = -drive.right.PID.output;
		}
		break;
	case Gyro:
		speed = MATH_map(speed, 127, 0, 240, 0);

		MATH_calculatePID(drive.left.PID, MATH_motionProfile(drive.left.motionProfile, abs(SensorValue[SENSOR_gyro]), pulses, speed), MATH_getSpeed(drive.left.previousPosition, abs(SensorValue[SENSOR_gyro])));

		MATH_calculatePID(drive.right.PID, MATH_motionProfile(drive.right.motionProfile, abs(SensorValue[SENSOR_gyro]), pulses, speed), MATH_getSpeed(drive.right.previousPosition, abs(SensorValue[SENSOR_gyro])));

		drive.left.output = drive.left.PID.output;
		drive.right.output = -drive.right.PID.output;
		break;

	default:
		if(abs(SensorValue[SENSOR_encoderL]) < pulses){
			drive.left.output = speed;
			drive.right.output = MATH_swingTurnInside(turnRadius, speed);
			if(pulses < 0){
				drive.left.output = -drive.left.PID.output;
				drive.right.output = -drive.right.PID.output;
			}
		}
		else{
			drive.left.output = drive.right.output = 0;
			drive.left.PID.notDone = drive.right.PID.notDone = false;
		}
	}
	//writeDebugStream("Converted Pulses=%f\t", pulses);	writeDebugStreamLine("Sensor L = %f, Outputs = %d",abs(SensorValue[SENSOR_encoderL]),drive.outputs[0]);

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.left.output;
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.right.output;
}

void DRIVE_turnLeft(ENUM_driveMode mode, float pulses, float turnRadius, byte speed){
	if(mode == Gyro){
		pulses = MATH_degreesToTicks(pulses);
	}
	else{
		pulses = MATH_degreesToPulses(pulses, turnRadius);
	}

	switch(mode){
	case PID:
		speed = MATH_map(speed, 127, 0, 240, 0);
		MATH_calculatePID(drive.right.PID, MATH_motionProfile(drive.right.motionProfile, abs(SensorValue[SENSOR_encoderR]), fabs(pulses), speed), MATH_getSpeed(drive.right.previousPosition, abs(SensorValue[SENSOR_encoderR])));
		MATH_calculatePID(drive.left.PID, MATH_swingTurnInside(turnRadius, MATH_motionProfile(drive.left.motionProfile, abs(SensorValue[SENSOR_encoderL]), fabs(pulses), speed)), MATH_getSpeed(drive.left.previousPosition, abs(SensorValue[SENSOR_encoderL])));

		if(pulses >= 0){
			drive.left.output = drive.left.PID.output;
			drive.right.output = drive.right.PID.output;
		}
		else if(pulses < 0){
			drive.left.output = -drive.left.PID.output;
			drive.right.output = -drive.right.PID.output;
		}
		break;
	case Gyro:
		speed = MATH_map(speed, 127, 0, 240, 0);
		MATH_calculatePID(drive.right.PID, MATH_motionProfile(drive.right.motionProfile, abs(SensorValue[SENSOR_gyro]), pulses, speed), MATH_getSpeed(drive.right.previousPosition, abs(SensorValue[SENSOR_gyro])));
		MATH_calculatePID(drive.left.PID, MATH_motionProfile(drive.left.motionProfile, abs(SensorValue[SENSOR_gyro]), pulses, speed), MATH_getSpeed(drive.left.previousPosition, abs(SensorValue[SENSOR_gyro])));

		drive.left.output = drive.left.PID.output;
		drive.right.output = -drive.right.PID.output;
		break;

	default:
		if(abs(SensorValue[SENSOR_encoderL]) < pulses){
			drive.left.output = speed;
			drive.right.output = MATH_swingTurnInside(turnRadius, speed);
			if(pulses < 0){
				drive.left.output = -drive.left.PID.output;
				drive.right.output = -drive.right.PID.output;
			}
		}
		else{
			drive.left.output = drive.right.output = 0;
			drive.left.PID.notDone = drive.right.PID.notDone = false;
		}
	}
	//writeDebugStream("Converted Pulses=%f\t", pulses);	writeDebugStreamLine("Sensor L = %f, Outputs = %d",abs(SensorValue[SENSOR_encoderL]),drive.outputs[0]);

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.left.output;
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.right.output;
}


//--Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake----Mobile Goal Intake//
void MOBILEGOAL_retract(bool retract){
	if(retract){
		MATH_calculatePID(mobileGoalIntake.PID, META_mogoRetracted, SensorValue[SENSOR_potMogo]);
		motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = -mobileGoalIntake.PID.output;
	}
	else{
		MATH_calculatePID(mobileGoalIntake.PID, META_mogoExtended, SensorValue[SENSOR_potMogo]);
		motor[MOTOR_mobileGoalL] = motor[MOTOR_mobileGoalR] = -mobileGoalIntake.PID.output;
	}
}

void MOBILEGOAL_operatorControl(bool notAnalog){
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
void ARM_move(ENUM_driveMode mode, ubyte position){
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
			if(SensorValue[SENSOR_potArm] < META_armUp) arm.output = -META_armMaxOutput;
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

void ARM_operatorControl(bool analog){
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
void GOLIATH_pickUp(bool pickUp, ubyte cycles){
	if(pickUp){
		motor[MOTOR_coneIntake] = -META_coneIntakeSpeed;
	}
	else{
		motor[MOTOR_coneIntake] = 127;
	}
	coneIntake.counter++;

	if(coneIntake.counter >= cycles){
		coneIntake.notDone = false;
		motor[MOTOR_coneIntake] = 0;
	}
}

void GOLIATH_operatorControl(){
	if(vexRT[JOYSTICK_coneIntakeD]){
		motor[MOTOR_coneIntake] = 127;
	}
	else if(vexRT[JOYSTICK_coneIntakeP]){
		motor[MOTOR_coneIntake] = -META_coneIntakeMaxOutput;
	}
	else{
		motor[MOTOR_coneIntake] = 0;
	}
}

#endif
