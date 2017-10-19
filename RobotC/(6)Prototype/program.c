#pragma config(Sensor, in2,    potL,           sensorPotentiometer)
#pragma config(Sensor, in3,    potR,           sensorPotentiometer)
#pragma config(Sensor, in8,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  encoderL,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  encoderR,       sensorQuadEncoder)
#pragma config(Motor,  port1,           mogoL,         tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           driveLF,       tmotorVex393HighSpeed_MC29, openLoop, encoderPort, dgtl1)
#pragma config(Motor,  port3,           driveLB,       tmotorVex393HighSpeed_MC29, openLoop, encoderPort, dgtl1)
#pragma config(Motor,  port4,           armL,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           clawL,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           clawR,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           armR,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           driveRB,       tmotorVex393HighSpeed_MC29, openLoop, encoderPort, dgtl3)
#pragma config(Motor,  port9,           driveRF,       tmotorVex393HighSpeed_MC29, openLoop, encoderPort, dgtl3)
#pragma config(Motor,  port10,          mogoR,         tmotorVex393_HBridge, openLoop)

#include "constants.h"

#pragma platform(VEX2)
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"
#include "functions.h"


void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;

	initialize();

}

task autonomous(){
	resetValues();

	writeDebugStreamLine("Begun autonomous");

	while (false) {
		drive(f, 12, 100);	//Move 12 inches forward
		writeDebugStream("Drive f 12 100, ");
		armsControl(u);	//Raise right arm and lower left
		writeDebugStream("Arm u, ");
		clawsControl(1);	//Open right claw and close left
		writeDebugStream("Claws 1, ");
		mobileGoalControl(0);	//Extend mobile goal intake
		writeDebugStream("Mogo 0");
	}
	resetValues();

	while (!driveDone || !armsDone || !clawsDone || !mogoDone) {
		drive(b, 12, 100);	//Move 12 inches backward
		armsControl(lr);	//Lower right arm to loader level and raise left
		clawsControl(0);	//Close right claw and open left
		mobileGoalControl(1);	//Retract mobile goal intake
	}
	resetValues();
}

task usercontrol(){
	while (true){
		//Drive
		driveOperatorControl();

		//Arm
		armsOperatorControl();

		//Claw
		clawsOperatorControl();

		//Mobile Goal
		mobileGoalOperatorControl();

		delay(DRIVERCONTROL_LOOP_DELAY);
	}
}
