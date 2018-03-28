/*   functions.h - Contains necessary functions to control robot            *
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

typedef struct{
	byte presentCurrent, desiredCurrent;
	short presentCycle, desiredCycle;
	float maxSlope;
}STRUCT_slewRate;

typedef struct {
	float KP, KI, KD, KV, integral, derivative;
	int error, lastError;
	byte output, correctionThreshold;
}STRUCT_PID;

typedef struct{
	float valuesHistory[5], sorted[5];
}STRUCT_medianFilter;

typedef struct{
	tSensors port;
	bool inverted;
	int currentPosition, previousPosition, pulsesPerRevolution;
	float RPM;
	STRUCT_medianFilter RPMfilter;
}STRUCT_SENSOR_encoder;

typedef struct{
	tSensors port[2];
	int intermediateReading;
	bool toggleReset;
}STRUCT_SENSOR_lineFollower;

typedef struct{
	byte output;
	STRUCT_slewRate slewRate;	STRUCT_PID PID; STRUCT_SENSOR_encoder encoder;
}STRUCT_driveSide;

typedef struct{
	byte joystickInputs[2];
	int previousGyro;	bool rectify;

	STRUCT_driveSide left;	STRUCT_driveSide right;
}STRUCT_drive;

STRUCT_drive drive;

#include "utils.h"

//Function prototype
void resetValues();

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


	for(int counter = 0; counter < 5; counter++){
		drive.left.encoder.RPMfilter.valuesHistory[counter] = 0;
		drive.left.encoder.RPMfilter.sorted[counter] = 0;
		drive.right.encoder.RPMfilter.valuesHistory[counter] = 0;
		drive.right.encoder.RPMfilter.sorted[counter] = 0;
	}

	slewRateInit(drive.left.slewRate, 0, 0, 0, 0, META_driveSlewRateMaxSlope);
	slewRateInit(drive.right.slewRate, 0, 0, 0, 0, META_driveSlewRateMaxSlope);
	pidInit(drive.left.PID, 0, 0, 0, 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
	pidInit(drive.right.PID, 0, 0, 0, 0, 0, 0, 0, 0, PID_correctionThresholdDrive);
	drive.left.PID.KV = drive.right.PID.KV = PID_KVdrive;

	drive.joystickInputs[0] = 0;
	drive.joystickInputs[1] = 0;
	drive.previousGyro = 0;
	drive.rectify = false;
}

void operatorControl(int Cycle){
	//Drive
	drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
	drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

	if((byte)checkWithinThreshold(drive.joystickInputs[0], JOYSTICK_driveThreshold, -JOYSTICK_driveThreshold))	drive.joystickInputs[0] = 0;
	if((byte)checkWithinThreshold(drive.joystickInputs[1], JOYSTICK_driveThreshold, -JOYSTICK_driveThreshold))	drive.joystickInputs[1] = 0;

	drive.left.output = clamp(drive.joystickInputs[0] + drive.joystickInputs[1]);
	drive.right.output = clamp(drive.joystickInputs[0] - drive.joystickInputs[1]);

	slewRateInit(drive.left.slewRate, motor[MOTOR_driveLF], drive.left.output, 1, 5, META_driveSlewRateMaxSlope);
	slewRateInit(drive.right.slewRate, motor[MOTOR_driveRF], drive.right.output, 1, 5, META_driveSlewRateMaxSlope);

	slewRateControl(MOTOR_driveLF, drive.left.slewRate);
	slewRateControl(MOTOR_driveLB, drive.left.slewRate);
	slewRateControl(MOTOR_driveRF, drive.right.slewRate);
	slewRateControl(MOTOR_driveRB, drive.right.slewRate);

	//Update Encoders
	updateEncoder(drive.left.encoder);
	updateEncoder(drive.right.encoder);

	writeDebugStream("speedPIDRobot(%f, %f, false)", medianFilter(drive.left.encoder.RPMfilter, drive.left.encoder.RPM), medianFilter(drive.right.encoder.RPMfilter, drive.right.encoder.RPM));

}

void speedPIDRobot(float driveLeftRPM, float driveRightRPM, bool bMobileGoalLoaded){
	mobileGoalLoaded(bMobileGoalLoaded, false, false, true);
	updateEncoder(drive.left.encoder);	updateEncoder(drive.right.encoder);

	drive.left.output = calculatePID(drive.left.PID, driveLeftRPM, drive.left.encoder.RPM) + drive.left.PID.KV * drive.left.encoder.RPM;
	drive.right.output = calculatePID(drive.right.PID, driveRightRPM, drive.right.encoder.RPM) + drive.right.PID.KV * drive.right.encoder.RPM;

	delay(META_loopsDelay);
}

#endif
