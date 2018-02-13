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
		mobileGoalOperatorControl(false);
		armOperatorControl(false);
		coneIntakeOperatorControl();
		LCD_calibrate();
		delay(META_loopsDelay);
	}
}

task autonomous(){
	//resetValues();
	initialize();

	resetValues();

	while(coneIntake.notDone){
		moveConeIntake(true, 50);
		delay(META_loopsDelay);
	}
	writeDebugStream("Cone Intake Pick Up");
	resetValues();

	while(coneIntake.notDone){
		moveConeIntake(false, 50);
	}
	writeDebugStream("Cone Intake deposit");
	resetValues();

	while(arm.PID.notDone){
		moveArm(1);
		delay(META_loopsDelay);
	}
	writeDebugStream("Arm done rising");

}
