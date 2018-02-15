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

	resetValues();
	loadMobileGoal(true, true, true);

	while(drive.PID.notDone){
		turnRight(Gyro, 90, META_driveWidth/2, 100);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, true, true);

	while(drive.PID.notDone){
		turnLeft(Gyro, 90, META_driveWidth/2, 100);
		delay(META_loopsDelay);
	}

}
