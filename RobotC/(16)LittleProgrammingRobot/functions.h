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
	byte presentCurrent;	int presentCycle;
	byte desiredCurrent;	int desiredCycle;
} STRUCT_slewRate;

typedef struct {
	float KP;		float KI;				float KD;
	int error;	float integral;	float derivative;
	int lastError;	byte output;
}STRUCT_PID;

typedef struct{
	byte output;
	int previousPosition;
	STRUCT_slewRate slewRate;	STRUCT_PID PID;
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

	drive.left.output = 0;
	drive.left.previousPosition = 0;
	drive.right.output = 0;
	drive.right.previousPosition = 0;

	slewRateInit(drive.left.slewRate, 0, 0, 0, 0);
	slewRateInit(drive.right.slewRate, 0, 0, 0, 0);
	pidInit(drive.left.PID, 0, 0, 0, 0, 0, 0, 0, 0);
	pidInit(drive.right.PID, 0, 0, 0, 0, 0, 0, 0, 0);\

	drive.joystickInputs[0] = 0;
	drive.joystickInputs[1] = 0;
	drive.previousGyro = 0;
	drive.rectify = false;
}

void operatorControl(int Cycle){
	//Drive
	drive.joystickInputs[0] = vexRT[JOYSTICK_driveF];
	drive.joystickInputs[1] = vexRT[JOYSTICK_driveS];

	if((byte)MATH_withinThreshold(drive.joystickInputs[0], JOYSTICK_driveThreshold, -JOYSTICK_driveThreshold))	drive.joystickInputs[0] = 0;
	if((byte)MATH_withinThreshold(drive.joystickInputs[1], JOYSTICK_driveThreshold, -JOYSTICK_driveThreshold))	drive.joystickInputs[1] = 0;

	drive.left.output = MATH_clamp(drive.joystickInputs[0] + drive.joystickInputs[1]);
	drive.right.output = MATH_clamp(drive.joystickInputs[0] - drive.joystickInputs[1]);

	slewRateInit(drive.left.slewRate, motor[MOTOR_driveLF], Cycle, drive.left.output, 15);
	slewRateInit(drive.right.slewRate, motor[MOTOR_driveRF], Cycle, drive.right.output, 15);

	slewRateControl(MOTOR_driveLF, drive.left.slewRate);
	slewRateControl(MOTOR_driveLB, drive.left.slewRate);
	slewRateControl(MOTOR_driveRF, drive.right.slewRate);
	slewRateControl(MOTOR_driveLB, drive.right.slewRate);

}

#endif
