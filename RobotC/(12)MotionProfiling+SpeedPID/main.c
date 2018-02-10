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

	drive.PID.timeout = 200;
	while(drive.PID.notDone){
		driveForward(PID, 24, 127);
		delay(META_loopsDelay);
	}
}
