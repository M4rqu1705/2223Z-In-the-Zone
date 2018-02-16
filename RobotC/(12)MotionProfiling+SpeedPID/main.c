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
	armLoaded(true);

	while(arm.PID.notDone){
		moveArm(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	armLoaded(false);

	while(arm.PID.notDone){
		moveArm(PID, 0);
		delay(META_loopsDelay);
	}

/*	//Flipout
	resetValues();
	armLoaded(true);

	while(arm.PID.notDone){
		moveArm(None, 1);
		moveConeIntake(true, 100);
		delay(META_loopsDelay);
	}

	resetValues();
	armLoaded(false);

	while(arm.PID.notDone){
		moveArm(None, 2);
		moveConeIntake(true, 100);
		delay(META_loopsDelay);
	}*/

	/*
	resetValues();
	loadMobileGoal(false, false, false);

	//Accelerate 6 inches towards the Mobile Goal
	while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
		driveForward(Acceleration, 6, 127);
		moveMobileGoal(false);
		moveArm(PID, 1);
		moveConeIntake(true, 100);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(false, false, false);

	//Move 45 inches towards the Mobile Goal
	while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
		driveForward(PID, 45, 127);
		moveMobileGoal(false);
		moveArm(PID, 1);
		moveConeIntake(true, 100);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, true, false);

	//Begin retracting Mobile Goal before beginning to accelerate for half a second
	for(int C = 0; C < 500/META_loopsDelay; C++){
		moveMobileGoal(true);
		moveArm(PID, 1);
		moveConeIntake(false, 100);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, true, false);

	//Accelerate 6 inches backwards and retract Mobile goal Intake
	while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
		driveBackwards(Acceleration, 6, 127);
		moveMobileGoal(true);
		moveArm(PID, 1);
		moveConeIntake(false, 100);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, true, false);

	/*
	//Move 45 inches backwards toward the 5 point zone.
	while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
		driveBackwards(PID, 45, 100);
		moveMobileGoal(true);
		moveArm(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, true, true);

	//Rotate 135 degrees to the left to align Buddy parallel to the starting bar
	while(drive.PID.notDone){
		turnLeft(Gyro, 140, 6.5, 100);
		moveArm(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, false, false);

	//Accelerate 6 inches moving towards the middle of the starting bar
	while(drive.PID.notDone){
		driveForward(Acceleration, 6, 127);
		moveArm(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, false, false);

	//Move 20 inches moving towards the middle of the starting bar
	while(drive.PID.notDone){
		driveForward(PID, 20, 100);
		moveArm(1);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, false, true);

	//Rotate left aligning Buddy perpendicular to the starting bar
	while(drive.PID.notDone){
		turnLeft(Gyro, 90, 6.5, 100);
		moveArm(1);
		delay(META_loopsDelay);
	}
	//Finish comment

	resetValues();
	loadMobileGoal(true, false, false);


	while(drive.PID.notDone){

	}

	resetValues();
	loadMobileGoal(true, false, false);

	while(drive.PID.notDone){
		driveForward(Acceleration, 20, 100);
		moveArm(1);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, false, false);

	while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
		driveForward(PID, 20, 100);
		moveMobileGoal(false);
		moveArm(1);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, true, false);

	while(drive.PID.notDone){
		driveBackwards(PID, 20, 100);
		moveArm(1);
		delay(META_loopsDelay);
	}

	resetValues();
	loadMobileGoal(true, false, false);

	while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
		driveBackwards(PID, 20, 100);
		moveMobileGoal(false);
		moveArm(1);
		delay(META_loopsDelay);
	}*/

}
