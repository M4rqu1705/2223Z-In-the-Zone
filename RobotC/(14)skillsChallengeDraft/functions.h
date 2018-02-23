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

	byte output;
	bool notDone;
}TEMPLATE_PID;

typedef struct {
	float distanceMultiplier[2];
	float offsets[2];

}TEMPLATE_motionProfile;

enum ENUM_driveMode{None = 0, PID, MtnPrfl, Gyro };


typedef struct{
	byte joystickInputs[2];
	float slewRateOutputs[2];
	byte output[2];
	float previousPosition[2];

	bool rectify;

	TEMPLATE_PID PID;
	TEMPLATE_PID swingTurnPID;
	TEMPLATE_motionProfile motionProfile;
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
void LOADED_mobileGoal(bool loaded, bool retract, bool usingGyro, bool speedPID);

void DRIVE_operatorControl(bool simple = false);
void DRIVE_forward(ENUM_driveMode mode, float pulses, float speed);
void DRIVE_backwards(ENUM_driveMode mode, float pulses, float speed);
void DRIVE_turnRight(ENUM_driveMode mode, float pulses, float turnRadius, float speed);
void DRIVE_turnLeft(ENUM_driveMode mode, float pulses, float turnRadius, float speed);

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
	if(bIfiRobotDisabled){
		SensorType[SENSOR_gyro] = sensorNone;
		wait1Msec(1000);
		SensorType[SENSOR_gyro] = sensorGyro;
		wait1Msec(2000);
		SensorValue[SENSOR_gyro] = 0;
		SensorScale[SENSOR_gyro] = 135;
		//SensorScale[SENSOR_gyro] = 141;
		SensorFullCount[SENSOR_gyro] = 3600;
	}

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

	motor[port1] = motor[port2] = motor[port3] = motor[port4] = motor[port5] =
	motor[port6] = motor[port7] = motor[port8] = motor[port9] = motor[port10] = 0;

	//Drive Values
	drive.joystickInputs[0] = drive.joystickInputs[1] = 0;
	drive.slewRateOutputs[0] = drive.slewRateOutputs[1] = 0;
	drive.output[0] = drive.output[1] = 0;
	drive.previousPosition[0] = drive.previousPosition[1] = 0;
	drive.rectify = false;

	drive.PID.error = 0;
	drive.PID.lastError = 0;
	drive.PID.integral = 0;
	drive.PID.integralMax = PID_integralMaxDrive;
	drive.PID.cyclesCounter = 0;
	drive.PID.output = 0;
	drive.PID.notDone = true;
	drive.PID.correctionCycles = PID_correctionCyclesDriveUnloaded;
	drive.PID.correctionThreshold = PID_correctionThresholdDriveUnloaded;

	drive.swingTurnPID.error = 0;
	drive.swingTurnPID.lastError = 0;
	drive.swingTurnPID.integral = 0;
	drive.swingTurnPID.integralMax = PID_integralMaxDrive;
	drive.swingTurnPID.cyclesCounter = 0;
	drive.swingTurnPID.output = 0;
	drive.swingTurnPID.notDone = false;
	drive.swingTurnPID.correctionCycles = PID_correctionCyclesDriveUnloaded;
	drive.swingTurnPID.correctionThreshold = PID_correctionThresholdDriveUnloaded;

	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.8;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 15;

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
	mobileGoalIntake.PID.correctionCycles = PID_correctionCyclesMobileGoalIntake;
	mobileGoalIntake.PID.correctionThreshold = PID_correctionThresholdMobileGoalIntake;

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
	arm.PID.correctionCycles = PID_correctionCyclesArm;
	arm.PID.correctionThreshold = PID_correctionThresholdArm;

	//Cone Intake Values
	coneIntake.notDone = true;
	coneIntake.counter = 0;
	coneIntake.output = 0;

	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	//SensorValue[SENSOR_gyro] = 0;
}

void LOADED_arm(bool loaded){
	if(loaded){
		arm.PID.KP = PID_KParm[1];
		arm.PID.KI = PID_KIarm[1];
		arm.PID.KD = PID_KDarm[1];
	}
	else{
		arm.PID.KP = PID_KParm[0];
		arm.PID.KI = PID_KIarm[0];
		arm.PID.KD = PID_KDarm[0];
	}
}

void LOADED_mobileGoal(bool loaded, bool retract, bool usingGyro, bool speedPID){
	//Adjust constants based on robot conditions and/or actions
	if(loaded){
		//If Mobile Goal is loaded
		if(usingGyro){
			//If drive will use Gyro
			if(speedPID){
				//If drive will use speed PID
				drive.PID.KP = PID_KPdriveGyro[1][1];
				drive.swingTurnPID.KP = PID_KPdriveGyro[1][1];
				drive.PID.KI = PID_KIdriveGyro[1][1];
				drive.swingTurnPID.KI = PID_KIdriveGyro[1][1];
				drive.PID.KD = PID_KDdriveGyro[1][1];
				drive.swingTurnPID.KD = PID_KDdriveGyro[1][1];
				drive.PID.correctionThreshold = drive.swingTurnPID.correctionThreshold = 0;
			}
			else{
				//If drive will use position PID
				drive.PID.KP = PID_KPdriveGyro[1][0];
				drive.swingTurnPID.KP = PID_KPdriveGyro[1][1];
				drive.PID.KI = PID_KIdriveGyro[1][0];
				drive.swingTurnPID.KI = PID_KIdriveGyro[1][1];
				drive.PID.KD = PID_KDdriveGyro[1][0];
				drive.swingTurnPID.KD = PID_KDdriveGyro[1][1];
				drive.swingTurnPID.correctionThreshold = 0;
			}
		}
		else{
			//If drive will not use Gyro
			if(speedPID){
				//If drive will use speed PID
				drive.PID.KP = PID_KPdrive[1][1];
				drive.swingTurnPID.KP = PID_KPdrive[1][1];
				drive.PID.KI = PID_KIdrive[1][1];
				drive.swingTurnPID.KI = PID_KIdrive[1][1];
				drive.PID.KD = PID_KDdrive[1][1];
				drive.swingTurnPID.KD = PID_KDdrive[1][1];
				drive.PID.correctionThreshold = drive.swingTurnPID.correctionThreshold = 0;
			}
			else{
				//If drive will use position PID
				drive.PID.KP = PID_KPdrive[1][0];
				drive.swingTurnPID.KP = PID_KPdrive[1][1];
				drive.PID.KI = PID_KIdrive[1][0];
				drive.swingTurnPID.KI = PID_KIdrive[1][1];
				drive.PID.KD = PID_KDdrive[1][0];
				drive.swingTurnPID.KD = PID_KDdrive[1][1];
				drive.swingTurnPID.correctionThreshold = 0;
			}
		}

		if(retract){
			//If Mobile Goal intake will retract
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntake[1][1];
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntake[1][1];
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntake[1][1];
		}
		else{
			//If Mobile Goal intake will extend
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntake[1][0];
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntake[1][0];
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntake[1][0];
		}
	}

	else{
		//If Mobile Goal is not loaded
		if(usingGyro){
			//If drive will use gyroscope
			if(speedPID){
				//If drive will use speed PID
				drive.PID.KP = PID_KPdriveGyro[0][1];
				drive.swingTurnPID.KP = PID_KPdriveGyro[0][1];
				drive.PID.KI = PID_KIdriveGyro[0][1];
				drive.swingTurnPID.KI = PID_KIdriveGyro[0][1];
				drive.PID.KD = PID_KDdriveGyro[0][1];
				drive.swingTurnPID.KD = PID_KDdriveGyro[0][1];
				drive.PID.correctionThreshold = drive.swingTurnPID.correctionThreshold = 0;
			}
			else{
				//If drive will use position PID
				drive.PID.KP = PID_KPdriveGyro[0][0];
				drive.swingTurnPID.KP = PID_KPdriveGyro[0][1];
				drive.PID.KI = PID_KIdriveGyro[0][0];
				drive.swingTurnPID.KI = PID_KIdriveGyro[0][1];
				drive.PID.KD = PID_KDdriveGyro[0][0];
				drive.swingTurnPID.KD = PID_KDdriveGyro[0][1];
				drive.swingTurnPID.correctionThreshold = 0;
			}
		}
		else{
			//If drive will use encoders instead of gyroscope
			if(speedPID){
				//If drive will use speed PID
				drive.PID.KP = PID_KPdrive[0][1];
				drive.swingTurnPID.KP = PID_KPdrive[0][1];
				drive.PID.KI = PID_KIdrive[0][1];
				drive.swingTurnPID.KI = PID_KIdrive[0][1];
				drive.PID.KD = PID_KDdrive[0][1];
				drive.swingTurnPID.KD = PID_KDdrive[0][1];
				drive.PID.correctionThreshold = drive.swingTurnPID.correctionThreshold = 0;
			}
			else{
				//If drive will use position PID
				drive.PID.KP = PID_KPdrive[0][0];
				drive.swingTurnPID.KP = PID_KPdrive[0][1];
				drive.PID.KI = PID_KIdrive[0][0];
				drive.swingTurnPID.KI = PID_KIdrive[0][1];
				drive.PID.KD = PID_KDdrive[0][0];
				drive.swingTurnPID.KD = PID_KDdrive[0][1];
				drive.swingTurnPID.correctionThreshold = 0;
			}
		}

		if(retract){
			//If mobile goal intake is unloaded and retracting
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntake[0][1];
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntake[0][1];
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntake[0][1];
		}
		else{
			//If mobile goal intake is unloaded and extending
			mobileGoalIntake.PID.KP = PID_KPmobileGoalIntake[0][0];
			mobileGoalIntake.PID.KI = PID_KImobileGoalIntake[0][0];
			mobileGoalIntake.PID.KD = PID_KDmobileGoalIntake[0][0];
		}
	}
}

//Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive -- Drive --//
void DRIVE_operatorControl(bool simple){

	drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
	drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

	if(simple){
		//Calculate output by first doing the operation and then clamping it while converting them to a byte
		drive.output[0] = (MATH_clamp(drive.joystickInputs[0] + drive.joystickInputs[1]));
		drive.output[1] = (MATH_clamp(drive.joystickInputs[0] - drive.joystickInputs[1]));

		//Assign calculated output values
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.output[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.output[1];
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
		drive.output[0] = MATH_clamp(MATH_round(drive.slewRateOutputs[0] + drive.slewRateOutputs[1]));
		drive.output[1] = MATH_clamp(MATH_round(drive.slewRateOutputs[0] - drive.slewRateOutputs[1]));

		//Move motors
		motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.output[0];
		motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.output[1];
	}
}

void DRIVE_forward(ENUM_driveMode mode, float pulses, float speed){
	pulses = MATH_inchesToPulses(pulses);

	switch(mode){
	case PID:
		//Position PID
		MATH_calculatePID(drive.PID, pulses, (abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2);
		//Assign PID outputs to variables
		drive.output[0] = drive.output[1] = drive.PID.output;
		//For debugging reasons
		writeDebugStreamLine("Drive forward PID: output = %f\toutput = %f", drive.output[0], drive.output[1]);
		break;

	case MtnPrfl:
		//Remap desired speed to the max speed the robot can drive at
		speed = MATH_map(speed, 127, 0, 240, 0);
		float temp = MATH_motionProfile(drive.motionProfile, ((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2), pulses, speed);
		MATH_calculatePID(drive.PID, temp, MATH_getSpeed(drive.previousPosition[1], abs(SensorValue[SENSOR_encoderR])));
		drive.output[0] = drive.output[1] = drive.PID.output;// + bias;
		if(temp == 0)	drive.PID.notDone = false;
		writeDebugStreamLine("Drive forward MtnPrfl: desiredSpeed = %f\tdesiredSpeedMtnProfile = %f\toutput = %f\toutput = %f",speed, temp, drive.output[0], drive.output[1]);
		break;

	default:
		if(((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2) < pulses){
			drive.output[0] = drive.output[1] = speed;
		}
		else{
			drive.output[0] = drive.output[1] = 0;
			drive.PID.notDone = false;
		}
	}

	//Rectify drive if necessary
	if((abs(SensorValue[SENSOR_encoderL])+abs(SensorValue[SENSOR_encoderR]))/2 < pulses*0.75 && drive.rectify){
		//If distance traveled is less than 3/4 of the distance desired
		if(pulses > MATH_inchesToPulses(50)){
			//Only use encoders over long distances
			if(abs(SensorValue[SENSOR_encoderL]) - abs(SensorValue[SENSOR_encoderR]) > 0) drive.output[1] += 1;
			else drive.output[1] -= 1;
		}
		else{		//Only use Gyro over short distances
			if(SensorValue[SENSOR_gyro]*0.1 < 10){
				//drive.output[0] -= SensorValue[SENSOR_gyro]*0.1;
				drive.output[1] += SensorValue[SENSOR_gyro]*0.1;
			}
		}
	}

	//Print values to datalog for debugging reasons
	datalogAddValue(0, drive.output[0]);
	datalogAddValue(1, drive.output[1]);

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.output[0];
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.output[1];
}

void DRIVE_backwards(ENUM_driveMode mode, float pulses, float speed){
	pulses = MATH_inchesToPulses(pulses);

	switch(mode){
	case PID:
		//Position PID
		MATH_calculatePID(drive.PID, pulses, (abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2);
		//Assign PID outputs to variables
		drive.output[0] = drive.output[1] = drive.PID.output;
		//For debugging reasons
		writeDebugStreamLine("Drive forward PID: output = %f\toutput = %f", drive.output[0], drive.output[1]);
		break;

	case MtnPrfl:
		//Remap desired speed to the max speed the robot can drive at
		speed = MATH_map(speed, 127, 0, 240, 0);
		float temp = MATH_motionProfile(drive.motionProfile, ((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2), pulses, speed);
		MATH_calculatePID(drive.PID, temp, MATH_getSpeed(drive.previousPosition[1], abs(SensorValue[SENSOR_encoderR])));
		drive.output[0] = drive.output[1] = drive.PID.output;// + bias;
		if(temp == 0)	drive.PID.notDone = false;
		writeDebugStreamLine("Drive forward MtnPrfl: desiredSpeed = %f\tdesiredSpeedMtnProfile = %f\toutput = %f\toutput = %f",speed, temp, drive.output[0], drive.output[1]);
		break;

	default:
		if(((abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2) < pulses){
			drive.output[0] = drive.output[1] = speed;
		}
		else{
			drive.output[0] = drive.output[1] = 0;
			drive.PID.notDone = false;
		}
	}

	//Rectify drive if necessary
	if((abs(SensorValue[SENSOR_encoderL])+abs(SensorValue[SENSOR_encoderR]))/2 < pulses*0.75 && drive.rectify){
		//If distance traveled is less than 3/4 of the distance desired
		if(pulses > MATH_inchesToPulses(50)){
			//Only use encoders over long distances
			if(abs(SensorValue[SENSOR_encoderL]) - abs(SensorValue[SENSOR_encoderR]) > 0) drive.output[1] += 1;
			else drive.output[1] -= 1;
		}
		else{		//Only use Gyro over short distances
			if(SensorValue[SENSOR_gyro]*0.1 < 10){
				//drive.output[0] -= SensorValue[SENSOR_gyro]*0.1;
				drive.output[1] += SensorValue[SENSOR_gyro]*0.1;
			}
		}
	}

	//Print values to datalog for debugging reasons
	datalogAddValue(0, drive.output[0]);
	datalogAddValue(1, drive.output[1]);

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = -drive.output[0];
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = -drive.output[1];
}

//Still not working*****************************************************************************************************
void DRIVE_turnLeft(ENUM_driveMode mode, float pulses, float turnRadius, float speed){
	pulses = (mode == Gyro) ? MATH_degreesToTicks(pulses) : MATH_degreesToPulses(pulses, turnRadius);

	switch(mode){
	case PID:
		//Position PID
		MATH_calculatePID(drive.PID, fabs(pulses), (abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2);
		//Speed PID
		MATH_calculatePID(drive.swingTurnPID, MATH_swingTurnInside(turnRadius, MATH_getSpeed(drive.previousPosition[1], abs(SensorValue[SENSOR_encoderR]))), MATH_getSpeed(drive.previousPosition[0], abs(SensorValue[SENSOR_encoderL])));
		drive.output[0] = drive.swingTurnPID.output;	drive.output[1] = drive.PID.output;
		//Invert direction if desired position is negative
		if(pulses < 0){	drive.output[0] = -drive.output[0];	drive.output[1] = -drive.output[1];}
		break;

	case MtnPrfl:
		int bias = floor(speed*0.5);
		//Remap desired speed to the max speed the robot can drive at
		speed = MATH_map(speed, 127, 0, 240, 0);
		//Get speed only once because, otherwise, the second time around it will output 0
		float temp = MATH_getSpeed(drive.previousPosition[1], abs(SensorValue[SENSOR_encoderR]));
		//Calculate right side speed (which is the fastest side)
		MATH_calculatePID(drive.PID, MATH_motionProfile(drive.motionProfile, abs(SensorValue[SENSOR_encoderR]), fabs(pulses), speed), temp);
		//Calculate the left side speed (which is the slowest side)
		MATH_calculatePID(drive.swingTurnPID, MATH_swingTurnInside(turnRadius, temp), MATH_getSpeed(drive.previousPosition[0], abs(SensorValue[SENSOR_encoderL])));
		//Assign outputs respectively
		drive.output[1] = drive.PID.output + bias;	drive.output[0] = drive.swingTurnPID.output + bias;
		//For debugging reasons
		writeDebugStreamLine("desired speed = %f,\tdesired Speed mtnprfl = %f,\toutput = %f,\toutput = %f", speed, temp, drive.output[0], drive.output[1]);
		//Invert the drive direction if the "pulses" input was originally negative
		if(pulses < 0){ drive.output[0] = -drive.output[0]; drive.output[1] = -drive.output[1];}
		break;

	case Gyro:
		//Point turn using position PID and Gyro
		MATH_calculatePID(drive.PID, pulses, SensorValue[SENSOR_gyro]);
		drive.output[0] = -drive.PID.output;	drive.output[1] = drive.PID.output;
		break;

	default:
		//By default, if the user doesn't choose either motion profiling, PID or gyro
		if(abs(SensorValue[SENSOR_encoderL]) < pulses){
			//Assign respective speeds using if destination not reached or surpassed
			drive.output[1] = speed;	drive.output[0] = MATH_swingTurnInside(turnRadius, speed);
			//Invert the drive direction if the pulses input was originally negative
			if(pulses < 0){	drive.output[0] = -drive.output[0];	drive.output[1] = -drive.output[1];}
		}
		else{
			//If the distance is reached, stop motors and determine that the drive is done
			drive.output[0] = drive.output[1] = 0;	drive.PID.notDone = drive.swingTurnPID.notDone = false;
		}
	}

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.output[0];
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.output[1];
}
//Still not working*****************************************************************************************************

void DRIVE_turnRight(ENUM_driveMode mode, float pulses, float turnRadius, float speed){
	pulses = (mode == Gyro) ? MATH_degreesToTicks(pulses) : MATH_degreesToPulses(pulses, turnRadius);

	switch(mode){
	case PID:
		//Position PID
		MATH_calculatePID(drive.PID, fabs(pulses), (abs(SensorValue[SENSOR_encoderL]) + abs(SensorValue[SENSOR_encoderR]))/2);
		//Speed PID
		MATH_calculatePID(drive.swingTurnPID, MATH_swingTurnInside(turnRadius, MATH_getSpeed(drive.previousPosition[0], abs(SensorValue[SENSOR_encoderL]))), MATH_getSpeed(drive.previousPosition[1], abs(SensorValue[SENSOR_encoderR])));
		drive.output[1] = drive.swingTurnPID.output;	drive.output[0] = drive.PID.output;
		//Invert direction if desired position is negative
		if(pulses < 0){	drive.output[0] = -drive.output[0];	drive.output[1] = -drive.output[1];}
		break;

	case MtnPrfl:
		//Remap desired speed to the max speed the robot can drive at
		speed = MATH_map(speed, 127, 0, 240, 0);
		//Get speed only once because, otherwise, the second time around it will output 0
		float temp = MATH_getSpeed(drive.previousPosition[0], abs(SensorValue[SENSOR_encoderL]));
		//Calculate right side speed (which is the fastest side)
		MATH_calculatePID(drive.PID, MATH_motionProfile(drive.motionProfile, abs(SensorValue[SENSOR_encoderL]), fabs(pulses), speed), temp);
		//Calculate the left side speed (which is the slowest side)
		MATH_calculatePID(drive.swingTurnPID, MATH_swingTurnInside(turnRadius, temp), MATH_getSpeed(drive.previousPosition[1], abs(SensorValue[SENSOR_encoderR])));
		//Assign outputs respectively
		drive.output[0] = drive.PID.output;	drive.output[1] = drive.swingTurnPID.output;
		//For debugging reasons
		writeDebugStreamLine("desired speed = %f,\tdesired Speed mtnprfl = %f,\toutput = %f,\toutput = %f", speed, temp, drive.output[0], drive.output[1]);
		//Invert the drive direction if the "pulses" input was originally negative
		if(pulses < 0){ drive.output[0] = -drive.output[0]; drive.output[1] = -drive.output[1];}
		break;

	case Gyro:
		//Point turn using position PID and Gyro
		MATH_calculatePID(drive.PID, pulses, SensorValue[SENSOR_gyro]);
		drive.output[0] = drive.PID.output;	drive.output[1] = -drive.PID.output;
		break;

	default:
		//By default, if the user doesn't choose either motion profiling, PID or gyro
		if(abs(SensorValue[SENSOR_encoderL]) < pulses){
			//Assign respective speeds using if destination not reached or surpassed
			drive.output[1] = speed;	drive.output[0] = MATH_swingTurnInside(turnRadius, speed);
			//Invert the drive direction if the pulses input was originally negative
			if(pulses < 0){	drive.output[0] = -drive.output[0];	drive.output[1] = -drive.output[1];}
		}
		else{
			//If the distance is reached, stop motors and determine that the drive is done
			drive.output[0] = drive.output[1] = 0;	drive.PID.notDone = drive.swingTurnPID.notDone = false;
		}
	}

	//Move motors
	motor[MOTOR_driveLF] = motor[MOTOR_driveLM] = motor[MOTOR_driveLB] = drive.output[0];
	motor[MOTOR_driveRF] = motor[MOTOR_driveRM] = motor[MOTOR_driveRB] = drive.output[1];
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
