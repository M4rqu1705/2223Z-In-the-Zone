#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"
#include "commons.h"
#include "MobileGoalAndDriveTestFunctions.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	initialize();
}

task usercontrol(){
	//startTask(autonomous);
	while (true){
		driveOperatorControl();
		//armsOperatorControl();
		mobileGoalOperatorControl();
		delay(LOOPS_DELAY);
	}
}

task autonomous(){

	while(drive.notDone){
		move(TurnRight,90, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(drive.notDone){
		move(TurnLeft, 180, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned left");

	while(drive.notDone){
		move(TurnRight, 90, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(drive.notDone){
		move(Backward, 5, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved backward");

	while(drive.notDone){
		move(Forward, 10, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved forward");

	while(drive.notDone){
		move(Backward, 5, 75);
		delay(1);
	}
	resetValues();
	writeDebugStreamLine("Done!!");
}
