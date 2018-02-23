/*   programmingSkills.h - Function containing motions of robot during programming skills*
*    Copyright (C) <2018>  Marcos Ricardo Pesante Colón                                  *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details.                                        *
*                                                                                        *
*    You should have received a copy of the GNU General Public License                   *
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.               */

#ifndef PROGRAMMING_SKILLS.h
#define PROGRAMMING_SKILLS.h

#pragma systemfile

void programmingSkillsAuton(){

	//Move forward 42 inches towards the first Mobile Goal
	resetValues();
	LOADED_mobileGoal(false, false, false, false);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.025;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 25;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<75; C++){
		DRIVE_forward(MtnPrfl, 42, 120);
		MOBILEGOAL_retract(false);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Begin retracting the Mobile Goal Intake for 100 milliseconds before driving back
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	for(int C = 0; C<=5; C++){
		MOBILEGOAL_retract(true);
		delay(META_loopsDelay);
	}

	//Move backwards 37 inches towards the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<75; C++){
		DRIVE_backwards(PID, 37, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Point turn 35 degrees inches backwards
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<40; C++){
		DRIVE_turnRight(Gyro, 35, 30, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Move backwards 32 inches parllel to the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_backwards(PID, 25, 100);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Swing turn 90 degrees right, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<40; C++){
		DRIVE_turnRight(Gyro, 125, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Move forward 42 inches towards the first Mobile Goal
	resetValues();
	LOADED_mobileGoal(false, false, false, false);
	LOADED_arm(true);
	//drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<75; C++){
		DRIVE_forward(PID, 30, 127);
		MOBILEGOAL_retract(false);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	// ------------------------- 20 points -------------------------/

	//Move backwards 20 inches from the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<75; C++){
		DRIVE_backwards(PID, 30, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Swing turn 90 degrees right, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(false, false, true, false);
	for(int C = 0; drive.PID.notDone && C<150; C++){
		DRIVE_turnRight(PID, 315, 6.5, 127);
		MOBILEGOAL_retract(false);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();

}
#endif
