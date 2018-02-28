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

#include "fast.h"

void programmingSkillsAuton(){

	FAST_pickup(PID, 42, 127, 50, -35, 127, 50, true);

	//Point turn 35 degrees right
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<30; C++){
		DRIVE_turnRight(Gyro, 35, 6.5, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Move backwards 32 inches parllel to the starting bar
	resetValues();
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	LOADED_mobileGoal(true, true, false, false);
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 30;
	drive.motionProfile.offsets[1] = 127;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<25; C++){
		DRIVE_backwards(MtnPrfl, 6, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_backwards(PID, 32, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Point turn 90 degrees right, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_turnRight(Gyro, 135, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Move forwards 30 inches parllel to the starting bar
	resetValues();
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	LOADED_mobileGoal(true, true, false, false);
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 30;
	drive.motionProfile.offsets[1] = 127;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<25; C++){
		DRIVE_forward(MtnPrfl, 6, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_forward(PID, 30, 127);
		MOBILEGOAL_retract(false);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	// ------------------------------------------ 20 points ---------------------------------------/

	//Move backwards 23 inches parllel to the starting bar
	resetValues();
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	LOADED_mobileGoal(true, true, false, false);
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 30;
	drive.motionProfile.offsets[1] = 127;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<25; C++){
		DRIVE_backwards(MtnPrfl, 6, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_backwards(PID, 24, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	SensorValue[SENSOR_gyro] = 0;

	//Point turn 90 degrees left, parallel to the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_turnLeft(Gyro, -90, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Move backwards 18 inches parllel to the starting bar
	resetValues();
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	LOADED_mobileGoal(true, true, false, false);
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 30;
	drive.motionProfile.offsets[1] = 127;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<25; C++){
		DRIVE_backwards(MtnPrfl, 6, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_backwards(PID, 18, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Point turn 90 degrees left, perpendicular to the starting bar and facing the second Mobile Goal
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_turnLeft(Gyro, -180, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	FAST_pickup(PID, 25, 40, 50, -25, 127, 50, true);

	//Point turn 135 degrees right, facing the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_turnRight(Gyro, -45, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	FAST_deposit(PID, 10, 127, 50, -15, 127, 50, true);

	//---------------------------------------- 30 points ------------------------------------------//

	SensorValue[SENSOR_gyro] = 0;


	/*resetValues();
	LOADED_mobileGoal(false, false, false, false);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.24;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 40;
	drive.motionProfile.offsets[1] = 127;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<100; C++){
	DRIVE_forward(MtnPrfl, 42, 120);
	MOBILEGOAL_retract(false);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Begin retracting the Mobile Goal Intake for 100 milliseconds before driving back
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	for(int C = 0; C<=10; C++){
	MOBILEGOAL_retract(true);
	delay(META_loopsDelay);
	}

	//Move backwards 35 inches towards the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = false;
	for(int C = 0; drive.PID.notDone && C<100; C++){
	DRIVE_backwards(MtnPrfl, 35, 120);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Point turn 40 degrees inches backwards
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<30; C++){
	DRIVE_turnRight(Gyro, 40, 30, 127);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Move backwards 23 inches parllel to the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<100; C++){
	DRIVE_backwards(MtnPrfl, 22, 120);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Point turn 90 degrees right, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
	DRIVE_turnRight(Gyro, 130, 6.5, 127);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Move forward 30 inches towards the first Mobile Goal
	resetValues();
	LOADED_mobileGoal(true, false, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.05;
	drive.motionProfile.distanceMultiplier[1] = 1;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<20; C++){
	DRIVE_forward(MtnPrfl, 30, 50);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Move forward 30 inches towards the first Mobile Goal
	resetValues();
	LOADED_mobileGoal(true, false, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<80; C++){
	DRIVE_forward(MtnPrfl, 30, 50);
	MOBILEGOAL_retract(false);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	// ------------------------- 20 points -------------------------/

	//Move backwards 20 inches from the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<100; C++){
	DRIVE_backwards(MtnPrfl, 20, 127);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	SensorValue[SENSOR_gyro] = 0;

	//Point turn 90 degrees right, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(false, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
	DRIVE_turnRight(Gyro, 95, 6.5, 127);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Move 3 inches forward, aligning Buddy perpendicularly with the second Mobile Goal
	resetValues();
	LOADED_mobileGoal(false, true, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<75; C++){
	DRIVE_forward(MtnPrfl, 3, 120);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}


	//Point turn 90 degrees right, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(false, false, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
	DRIVE_turnRight(Gyro, 180, 6.5, 127);
	MOBILEGOAL_retract(false);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Move forward 25 inches towards the second Mobile Goal
	resetValues();
	LOADED_mobileGoal(false, false, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<100; C++){
	DRIVE_forward(MtnPrfl, 25, 120);
	MOBILEGOAL_retract(false);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Begin retracting the Mobile Goal Intake for 300 milliseconds before driving back
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	for(int C = 0; C<=15; C++){
	MOBILEGOAL_retract(true);
	delay(META_loopsDelay);
	}

	//Move backwards 20 inches to the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<75; C++){
	DRIVE_backwards(MtnPrfl, 20, 120);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	SensorValue[SENSOR_gyro] = 0;

	//Point turn 135 degrees right, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(false, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
	DRIVE_turnRight(Gyro, 135, 6.5, 127);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Move forward 13 inches scoring the second Mobile Goal
	resetValues();
	LOADED_mobileGoal(true, false, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<50; C++){
	DRIVE_forward(MtnPrfl, 13, 120);
	MOBILEGOAL_retract(false);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	// ---------------------------------- 32 points ------------------------------------//

	//Move backwards 40 inches towards the starting bar
	resetValues();
	LOADED_mobileGoal(true, false, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = false;
	for(int C = 0; drive.PID.notDone && C<100; C++){
	DRIVE_backwards(MtnPrfl, 40, 120);
	MOBILEGOAL_retract(false);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	SensorValue[SENSOR_gyro] = 0;

	//Point turn 145 degrees right, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(false, false, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
	DRIVE_turnRight(Gyro, 125, 6.5, 127);
	MOBILEGOAL_retract(false);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Move forward 12 inches towards the third Mobile Goal
	resetValues();
	LOADED_mobileGoal(false, false, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<100; C++){
	DRIVE_forward(MtnPrfl, 12, 120);
	MOBILEGOAL_retract(false);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Begin retracting the Mobile Goal Intake for 300 milliseconds before driving back
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	for(int C = 0; C<=15; C++){
	MOBILEGOAL_retract(true);
	delay(META_loopsDelay);
	}

	//Move backwards 30 inches to the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<75; C++){
	DRIVE_backwards(MtnPrfl, 20, 120);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Point turn 145 degrees left, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
	DRIVE_turnLeft(Gyro, 0, 6.5, 127);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	//Move forward 40 inches towards the third Mobile Goal
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<100; C++){
	DRIVE_forward(MtnPrfl, 40, 120);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}

	resetValues();
	LOADED_mobileGoal(true, false, false, true);

	for(int C = 0; mobileGoalIntake.PID.notDone && C<100; C++){
	MOBILEGOAL_retract(false);
	delay(META_loopsDelay);
	}

	//Move backwards 40 inches towards the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, false, true);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.75;
	drive.motionProfile.offsets[0] = 25;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = false;
	for(int C = 0; drive.PID.notDone && C<100; C++){
	DRIVE_backwards(MtnPrfl, 40, 120);
	MOBILEGOAL_retract(true);
	ARM_move(PID, 1);
	delay(META_loopsDelay);
	}*/
}

#endif
