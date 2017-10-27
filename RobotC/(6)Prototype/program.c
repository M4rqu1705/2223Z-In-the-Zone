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

	while(!driveDone){
		drive(r,90, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(!driveDone){
		drive(l,180, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned left");

	while(!driveDone){
		drive(r,90, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(!driveDone){
		drive(b,5, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved backward");

	while(!driveDone){
		drive(f,10, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved forward");

	while(!driveDone){
		drive(b,5, 75, false);
		delay(1);
	}
	resetValues();
	writeDebugStreamLine("Done!!");
}
