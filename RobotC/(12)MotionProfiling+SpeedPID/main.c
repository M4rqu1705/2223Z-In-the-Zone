#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"
#include "commons.h"
#include "functions.h"
//#include "programmingSkills.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	initialize();
}

task usercontrol(){
	//startTask(autonomous);
	while (true){
		driveOperatorControl(false);
		delay(META_loopsDelay);

	}
}

task autonomous(){
	//resetValues();
	initialize();

	while(drive.PID.notDone){
		turnLeft(PID, 90, 20, 50);
		delay(META_loopsDelay);
	}
	resetValues();
	writeDebugStreamLine("Turned 90 degrees without turning radius");

	delay(1000);

	while(drive.PID.notDone){
		turnRight(PID, 90, 20, 50);
		delay(META_loopsDelay);
	}
	resetValues();
	writeDebugStreamLine("Turned 90 degrees with turning radius of 13\"");

}
