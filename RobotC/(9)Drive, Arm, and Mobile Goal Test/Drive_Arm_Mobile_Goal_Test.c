#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"
#include "commons.h"
#include "MobileGoalAndDriveTestFunctions2.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	initialize();
}

task usercontrol(){
	//startTask(autonomous);
	while (true){
		driveOperatorControl();
		mobileGoalOperatorControl(false);
		delay(LOOPS_DELAY);
	}
}

task autonomous(){
	resetValues();
	clearDebugStream();

	if(mobileGoalIntake.notDone){
		moveMobileGoal(0);
	}
	resetValues();
	writeDebugStreamLine("Extended Mobile Goal Intake");

	if(mobileGoalIntake.notDone){
		moveMobileGoal(1);
	}
	resetValues();
	writeDebugStreamLine("Retracted Mobile Goal Intake");
}
