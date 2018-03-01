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

	FAST_pickup(PID, 45, 127, 50, -38, 127, 52, false);

	//Point turn 35 degrees left
	resetValues();
	LOADED_mobileGoal(true, false, true, false);
	for(int C = 0; drive.PID.notDone && C<30; C++){
		DRIVE_turnLeft(Gyro, -35, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Move backwards 30 inches parllel to the starting bar
	resetValues();
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	LOADED_mobileGoal(true, false, false, false);
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 30;
	drive.motionProfile.offsets[1] = 127;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<25; C++){
		DRIVE_backwards(MtnPrfl, 6, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	for(int C = 0; drive.PID.notDone && C<45; C++){
		DRIVE_backwards(PID, 30, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Point turn 90 degrees left, perpendicular to the starting bar
	resetValues();
	LOADED_mobileGoal(true, false, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_turnLeft(Gyro, -135, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Move forwards 30 inches perpendicular to the starting bar
	resetValues();
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	LOADED_mobileGoal(true, false, false, false);
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 30;
	drive.motionProfile.offsets[1] = 127;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<25; C++){
		DRIVE_forward(MtnPrfl, 6, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	for(int C = 0; drive.PID.notDone && C<45; C++){
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
		MOBILEGOAL_retract(false);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	for(int C = 0; drive.PID.notDone && C<45; C++){
		DRIVE_backwards(PID, 24, 127);
		MOBILEGOAL_retract(true);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	SensorValue[SENSOR_gyro] = 0;

	//Point turn 90 degrees left, parallel to the starting bar
	resetValues();
	LOADED_mobileGoal(false, true, true, false);
	for(int C = 0; drive.PID.notDone && C<40; C++){
		DRIVE_turnLeft(Gyro, -90, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Move backwards 18 inches parllel to the starting bar
	resetValues();
	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	LOADED_mobileGoal(false, true, false, false);
	drive.motionProfile.distanceMultiplier[0] = 0.25;
	drive.motionProfile.distanceMultiplier[1] = 1.5;
	drive.motionProfile.offsets[0] = 30;
	drive.motionProfile.offsets[1] = 127;
	drive.rectify = true;
	for(int C = 0; drive.PID.notDone && C<25; C++){
		DRIVE_backwards(MtnPrfl, 6, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_backwards(PID, 18, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	//Point turn 90 degrees left, perpendicular to the starting bar and facing the second Mobile Goal
	resetValues();
	LOADED_mobileGoal(false, true, true, false);
	for(int C = 0; drive.PID.notDone && C<40; C++){
		DRIVE_turnLeft(Gyro, -180, 6.5, 127);
		ARM_move(PID, 1);
		MOBILEGOAL_retract(false);
		delay(META_loopsDelay);
	}

	FAST_pickup(PID, 25, 127, 50, -25, 127, 50, true);

	//Point turn 180 degrees right, facing the starting bar
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_turnRight(Gyro, 0, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	FAST_deposit(PID, 12, 127, 50, -43, 127, 75, true);

	//---------------------------------------- 30 points ------------------------------------------//

	//Turn left 90 degrees to face third mobile goal
	resetValues();
	LOADED_mobileGoal(false, false, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_turnLeft(Gyro, -90, 6.5, 127);
		ARM_move(PID, 1);
		MOBILEGOAL_retract(false);
		delay(META_loopsDelay);
	}

	//Pick up third mobile goal
	FAST_pickup(PID, 25, 127, 35, 7, 127, 30, true);

	//Turn right 90 degrees ready to deposit mobile goal
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<40; C++){
		DRIVE_turnRight(Gyro, 0, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();

	//Move forward 30 inches perpendicular to the starting bar
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
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_forward(PID, 30, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	FAST_deposit(PID, 14, 127, 45, -14, 127, 60, true);

	//------------------------------------------- 40 points ----------------------------------------------//

	SensorValue[SENSOR_gyro] = 0;

	//Turn left 130 degrees for positioning
	resetValues();
	SensorValue[SENSOR_gyro] = 0;
	LOADED_mobileGoal(false, false, true, false);
	for(int C = 0; drive.PID.notDone && C<50; C++){
		DRIVE_turnLeft(Gyro, -117.5, 6.5, 127);
		ARM_move(PID, 1);
		MOBILEGOAL_retract(false);
		delay(META_loopsDelay);
	}

	FAST_pickup(PID, 50, 127, 50, -45, 127, 50, true);

	//Turn right 130 degrees for positioning
	resetValues();
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<55; C++){
		DRIVE_turnRight(Gyro, 45, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();

	FAST_deposit(PID, 15, 127, 50, -8.5, 127, 50, true);

	//--------------------------------- 50 points ---------------------------------------//

	//Turn right 130 degrees for positioning
	resetValues();
	SensorValue[SENSOR_gyro] = 0;
	LOADED_mobileGoal(true, true, true, false);
	for(int C = 0; drive.PID.notDone && C<55; C++){
		DRIVE_turnRight(Gyro, 125, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}

	resetValues();
	FAST_pickup(PID, 50, 127, 75, 48, 127, 75, true);

	resetValues();

}
#endif
