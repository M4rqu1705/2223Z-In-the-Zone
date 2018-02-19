/*   main.c - Main file controlling robot movement, containing its motions  *
*    Copyright (C) <2018>  Marcos Ricardo Pesante Colón                     *
*                                                                           *
*    This program is free software: you can redistribute it and/or modify   *
*    it under the terms of the GNU General Public License as published by   *
*    the Free Software Foundation, either version 3 of the License, or      *
*    (at your option) any later version.                                    *
*                                                                           *
*    This program is distributed in the hope that it will be useful,        *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of         *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
*    GNU General Public License for more details.                           *
*                                                                           *
*    You should have received a copy of the GNU General Public License      *
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"
#include "commons.h"
#include "functions.h"
#include "programmingSkills.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	initialize();


	//Only include piece of code if META_usingLCD is defined
#ifdef META_usingLCD
	do{
		if(bIfiRobotDisabled)	LCD_select();
		else break;
	}while (!lcdReady);

	//Clear the LCD
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDCenteredString(0, "     2223-Z     ");    //Output 2223-Z on the screen, signaling that the lcd is done
#endif
}

task usercontrol(){
	while (true){
		driveOperatorControl(false);
		mobileGoalOperatorControl(false);
		armOperatorControl(false);
		coneIntakeOperatorControl();
		//LCD_calibrate();
		delay(META_loopsDelay);
	}
}

task autonomous(){
	if(currentSideRight){
		switch(currentCode){
		case twentyPointMogo:
			resetValues();
			loadMobileGoal(false, false, false);
			armLoaded(true);

			//Accelerate 3 inches towards the Mobile Goal
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveMobileGoal(false);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(false, false, false);
			drive.PID.timeout = 75;

			//Move 40 inches towards the Mobile Goal
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveForward(PID, 47, 127);
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
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);

			drive.PID.timeout = 50;

			//Move 42 inches backwards toward the 5 point zone.
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveBackwards(PID, 42, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();

			delay(250);

			//Deposit cone on Mobile Goal
			while(coneIntake.notDone){
				moveConeIntake(false, 500/META_loopsDelay);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, true);
			drive.PID.timeout = 50;

			//Rotate 135 degrees to the left to align Buddy parallel to the starting bar
			while(drive.PID.notDone){
				turnLeft(Gyro, 135, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Accelerate 3 inches moving towards the middle of the starting bar
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			//Move 23 inches moving towards the middle of the starting bar
			while(drive.PID.notDone){
				driveForward(PID, 23, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, true);

			drive.PID.timeout = 50;

			//Rotate left aligning Buddy perpendicular to the starting bar
			while(drive.PID.notDone){
				turnLeft(Gyro, 90, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Accelerate 5 inches towards the 20-point zone in order to score smoother
			while(drive.PID.notDone){
				driveForward(Acceleration, 5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			//Drive towards the 20 point zone and extend Mobile Goal Intake to score 20 points
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveForward(PID, 25, 100);
				moveMobileGoal(false);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);
			drive.PID.timeout = 50;

			//Move backwards while retracting Mobile Goal Intake to finish scoring
			while(drive.PID.notDone){
				driveBackwards(PID, 25, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			break;
		case tenPointMogo:
			resetValues();
			loadMobileGoal(false, false, false);
			armLoaded(true);

			//Accelerate 3 inches towards the Mobile Goal
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveMobileGoal(false);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(false, false, false);
			drive.PID.timeout = 75;

			//Move 40 inches towards the Mobile Goal
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveForward(PID, 47, 127);
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
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);

			drive.PID.timeout = 50;

			//Move 43 inches backwards toward the 5 point zone.
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveBackwards(PID, 42, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();

			delay(250);

			while(coneIntake.notDone){
				moveConeIntake(false, 500/META_loopsDelay);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, true);
			drive.PID.timeout = 50;

			//Rotate 135 degrees to the left to align Buddy parallel to the starting bar
			while(drive.PID.notDone){
				turnLeft(Gyro, 135, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Accelerate 6 inches moving towards the middle of the starting bar
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			//Move 20 inches moving towards the middle of the starting bar
			while(drive.PID.notDone){
				driveForward(PID, 23, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, true);

			drive.PID.timeout = 50;

			//Rotate left aligning Buddy perpendicular to the starting bar
			while(drive.PID.notDone){
				turnLeft(Gyro, 90, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			while(drive.PID.notDone){
				driveForward(Acceleration, 5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			while(mobileGoalIntake.PID.notDone){
				moveMobileGoal(false);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);
			drive.PID.timeout = 50;

			while(drive.PID.notDone){
				driveBackwards(PID, 25, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}
			resetValues();
			break;
		case fivePointMogo:
			resetValues();
			loadMobileGoal(false, false, false);
			armLoaded(true);

			//Accelerate 3 inches towards the Mobile Goal
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveMobileGoal(false);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(false, false, false);
			drive.PID.timeout = 75;

			//Move 40 inches towards the Mobile Goal
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveForward(PID, 47, 127);
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
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);

			drive.PID.timeout = 50;

			//Move 43 inches backwards toward the 5 point zone.
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveBackwards(PID, 42, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();

			delay(250);

			while(coneIntake.notDone){
				moveConeIntake(false, 500/META_loopsDelay);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, true);
			drive.PID.timeout = 50;

			//Rotate 180 degrees to the left to align Buddy at 45 degrees from the starting bar, parallel to the field
			while(drive.PID.notDone){
				turnLeft(Gyro, 180, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Accelerate 3 inches backwards to make enough space for the Mobile Goal
			while(drive.PID.notDone){
				driveBackwards(Acceleration, 3, 127);
				moveMobileGoal(false);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Extend Mobile goal Intake
			while(mobileGoalIntake.PID.notDone){
				moveMobileGoal(false);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			//Move 23 inches moving backwards to release the Mobile Goal
			while(drive.PID.notDone){
				driveBackwards(PID, 23, 100);
				moveMobileGoal(false);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			break;

		case programmingSkillsCode:
			programmingSkillsAuton();
			break;
		}
	}
	else{
		switch(currentCode){
		case twentyPointMogo:
			resetValues();
			loadMobileGoal(false, false, false);
			armLoaded(true);

			//Accelerate 3 inches towards the Mobile Goal
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveMobileGoal(false);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(false, false, false);
			drive.PID.timeout = 75;

			//Move 40 inches towards the Mobile Goal
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveForward(PID, 47, 127);
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
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);

			drive.PID.timeout = 50;

			//Move 42 inches backwards toward the 5 point zone.
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveBackwards(PID, 42, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();

			delay(250);

			//Deposit cone on Mobile Goal
			while(coneIntake.notDone){
				moveConeIntake(false, 500/META_loopsDelay);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, true);
			drive.PID.timeout = 50;

			//Rotate 135 degrees to the right to align Buddy parallel to the starting bar
			while(drive.PID.notDone){
				turnRight(Gyro, 135, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Accelerate 3 inches moving towards the middle of the starting bar
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			//Move 23 inches moving towards the middle of the starting bar
			while(drive.PID.notDone){
				driveForward(PID, 23, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, true);

			drive.PID.timeout = 50;

			//Rotate left aligning Buddy perpendicular to the starting bar
			while(drive.PID.notDone){
				turnRight(Gyro, 90, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Accelerate 5 inches towards the 20-point zone in order to score smoother
			while(drive.PID.notDone){
				driveForward(Acceleration, 5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			//Drive towards the 20 point zone and extend Mobile Goal Intake to score 20 points
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveForward(PID, 25, 100);
				moveMobileGoal(false);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);
			drive.PID.timeout = 50;

			//Move backwards while retracting Mobile Goal Intake to finish scoring
			while(drive.PID.notDone){
				driveBackwards(PID, 25, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			break;
		case tenPointMogo:
			resetValues();
			loadMobileGoal(false, false, false);
			armLoaded(true);

			//Accelerate 3 inches towards the Mobile Goal
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveMobileGoal(false);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(false, false, false);
			drive.PID.timeout = 75;

			//Move 40 inches towards the Mobile Goal
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveForward(PID, 47, 127);
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
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);

			drive.PID.timeout = 50;

			//Move 43 inches backwards toward the 5 point zone.
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveBackwards(PID, 42, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();

			delay(250);

			while(coneIntake.notDone){
				moveConeIntake(false, 500/META_loopsDelay);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, true);
			drive.PID.timeout = 50;

			//Rotate 135 degrees to the right to align Buddy parallel to the starting bar
			while(drive.PID.notDone){
				turnRight(Gyro, 135, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Accelerate 6 inches moving towards the middle of the starting bar
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			//Move 20 inches moving towards the middle of the starting bar
			while(drive.PID.notDone){
				driveForward(PID, 23, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, true);

			drive.PID.timeout = 50;

			//Rotate right aligning Buddy perpendicular to the starting bar
			while(drive.PID.notDone){
				turnRight(Gyro, 90, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			while(drive.PID.notDone){
				driveForward(Acceleration, 5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			while(mobileGoalIntake.PID.notDone){
				moveMobileGoal(false);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);
			drive.PID.timeout = 50;

			while(drive.PID.notDone){
				driveBackwards(PID, 25, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}
			resetValues();
			break;
		case fivePointMogo:
			resetValues();
			loadMobileGoal(false, false, false);
			armLoaded(true);

			//Accelerate 3 inches towards the Mobile Goal
			while(drive.PID.notDone){
				driveForward(Acceleration, 3, 127);
				moveMobileGoal(false);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(false, false, false);
			drive.PID.timeout = 75;

			//Move 40 inches towards the Mobile Goal
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveForward(PID, 47, 127);
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
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, false);

			drive.PID.timeout = 50;

			//Move 43 inches backwards toward the 5 point zone.
			while(drive.PID.notDone || mobileGoalIntake.PID.notDone){
				driveBackwards(PID, 42, 100);
				moveMobileGoal(true);
				moveArm(PID, 1);
				moveConeIntake(true, 100);
				delay(META_loopsDelay);
			}

			resetValues();

			delay(250);

			while(coneIntake.notDone){
				moveConeIntake(false, 500/META_loopsDelay);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, true, true);
			drive.PID.timeout = 50;

			//Rotate 180 degrees to the left to align Buddy at 45 degrees from the starting bar, parallel to the field
			while(drive.PID.notDone){
				turnRight(Gyro, 180, 6.5, 100);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Accelerate 3 inches backwards to make enough space for the Mobile Goal
			while(drive.PID.notDone){
				driveBackwards(Acceleration, 3, 127);
				moveMobileGoal(false);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);

			//Extend Mobile goal Intake
			while(mobileGoalIntake.PID.notDone){
				moveMobileGoal(false);
				delay(META_loopsDelay);
			}

			resetValues();
			loadMobileGoal(true, false, false);
			drive.PID.timeout = 50;

			//Move 23 inches moving backwards to release the Mobile Goal
			while(drive.PID.notDone){
				driveBackwards(PID, 23, 100);
				moveMobileGoal(false);
				moveArm(PID, 1);
				delay(META_loopsDelay);
			}

			resetValues();
			break;

		case programmingSkillsCode:
			programmingSkillsAuton();
			break;
		}
	}
}
