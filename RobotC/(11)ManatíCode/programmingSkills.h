#ifndef PROGRAMMING_SKILLS.h
#define PROGRAMMING_SKILLS.h

#pragma systemfile

void programmingSkillsAuton(){

	//First forward, move 45 inches
	while(drive.notDone){
		move(Forward, 25, 127);
		moveMobileGoal(0);
	}
	resetValues();

	//First forward, move 45 inches
	while(drive.notDone || mobileGoalIntake.notDone){
		move(Forward, 20, 60);
		moveMobileGoal(0);
	}
	resetValues();

	//FIrst retract, retract Mobile Goal
	while(mobileGoalIntake.notDone){
		moveMobileGoal(1);
	}
	resetValues();

	//First backward, move 35 inches
	while(drive.notDone){
		move(Backward, 35, 50);
	}
	resetValues();

	//First turn, rotate 135 degrees right
	while(drive.notDone){
		move(TurnRight, 135, 80);
	}
	resetValues();

	//Second forward, move 30 inches forward, parallel to starting bar
	while(drive.notDone){
		move(Forward, 30, 50);
	}
	resetValues();

	//Second turn, rotate 90 degrees right
	while(drive.notDone){
		move(TurnRight, 90, 75);
	}
	resetValues();

	//Move forward and deposit Mobile Goal
	PID_KIdrive = 1;
	while(drive.notDone || mobileGoalIntake.notDone){
		move(Forward, 22, 100);
		moveMobileGoal(0);
	}
	resetValues();
	PID_KIdrive = 0.5;

	//Move backwards, towards stationary goal
	while(drive.notDone){
		move(Backward, 23, 100);
		moveMobileGoal(1);
	}
	resetValues();
	//*******************************END 20 point Zone *********************************************//

	/*//Move back towards the wall
	while(drive.notDone){
		move(Backward, 33, 80);
		moveArm(1);
	}
	resetValues();

	while(drive.notDone){
		move(TurnLeft, 65, 50);
		moveArm(1);
	}
	resetValues();

	for(int C = 0; C<2000; C++){
		move(Backward, 12, 127);
		moveArm(1);
		delay(1);
	}
	resetValues();

	while(drive.notDone){
		move(Forward, 22, 50);
	}
	resetValues();

	while(drive.notDone || mobileGoalIntake.notDone){
		move(TurnLeft, 45, 50);
		moveMobileGoal(0);
		moveArm(0);
	}
	resetValues();*/

	while(drive.notDone){
		move(TurnLeft, 90, 50);
	}
	resetValues();

	while(drive.notDone){
		move(Backward, 20, 50);
	}
	resetValues();

	while(drive.notDone||mobileGoalIntake.notDone){
		move(TurnLeft, 90, 50);
		moveMobileGoal(0);
	}
	resetValues();

	//************************Ready to get second Mogo********************************//
	while(drive.notDone){
		move(Forward, 20, 50);
		moveMobileGoal(0);
	}
	resetValues();

	while(mobileGoalIntake.notDone){
		moveMobileGoal(1);
	}
	resetValues();

	while(drive.notDone){
		move(TurnRight, 160, 75);
	}
	resetValues();

	while(drive.notDone || mobileGoalIntake.notDone){
		move(Forward, 26, 127);
		moveMobileGoal(0);
	}
	resetValues();

	while(drive.notDone){
		move(Backward, 10, 127);
	}
	resetValues();

	while(drive.notDone){
		move(TurnLeft, 45, 50);
	}
	resetValues();

	while(drive.notDone){
		move(Backward, 25, 127);
	}
	resetValues();
/*
	//Remove Mogo intake from mogo in 10 point zone
	while(drive.notDone){
		move(TurnLeft, 85, 50);
		moveMobileGoal(1);
	}
	resetValues();

	while(drive.notDone){
		moveMobileGoal(1);
		move(Forward, 45, 50);
	}
	resetValues();

	while(drive.notDone){
		move(TurnLeft, 45, 50);
		moveMobileGoal(0);
	}
	resetValues();

	//Move forward to pick third Mogo
	while(drive.notDone){
		move(Forward, 45, 80);
		moveMobileGoal(0);
	}
	resetValues();

	while(mobileGoalIntake.notDone){
		moveMobileGoal(1);
	}
	resetValues();

	while(drive.notDone){
		move(Backward, 30, 50);
	}
	resetValues();

	while(drive.notDone){
		move(TurnLeft, 180, 75);
	}
	resetValues();

	while(drive.notDone || mobileGoalIntake.notDone){
		move(Forward, 10, 75);
		moveMobileGoal(0);
	}
	*/
}

#endif
