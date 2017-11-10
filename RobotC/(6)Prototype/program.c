#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"
#include "constants.h"
#include "functions.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	initialize();
}

task usercontrol(){
	//startTask(autonomous);
	while (true){
		driveOperatorControl();
		armsOperatorControl();
		clawsOperatorControl();
		mobileGoalOperatorControl();
		delay(DRIVERCONTROL_LOOP_DELAY);
	}
}

task autonomous(){

	while(!driveNotDone){
		drive(r,90, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(driveNotDone){
		drive(l,180, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned left");

	while(driveNotDone){
		drive(r,90, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(driveNotDone){
		drive(b,5, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved backward");

	while(driveNotDone){
		drive(f,10, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved forward");

	while(driveNotDone){
		drive(b,5, 75, false);
		delay(1);
	}
	resetValues();
	writeDebugStreamLine("Done!!");
}
