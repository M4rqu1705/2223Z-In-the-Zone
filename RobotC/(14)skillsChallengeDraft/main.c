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
	LCD_init();
}

task usercontrol(){
	while (true){
		DRIVE_operatorControl(false);
		MOBILEGOAL_operatorControl(false);
		ARM_operatorControl(false);
		GOLIATH_operatorControl();
		LCD_calibrate();
		//writeDebugStreamLine("Left Speed = %f,\tRight Speed = %f", MATH_getSpeed(drive.previousPosition[0], abs(SensorValue[SENSOR_encoderL])), MATH_getSpeed(drive.previousPosition[1], abs(SensorValue[SENSOR_encoderR])));
		delay(META_loopsDelay);
	}
}

task autonomous(){
	resetValues();
	LOADED_mobileGoal(false, false, false, false);
	LOADED_arm(true);
	drive.motionProfile.distanceMultiplier[0] = 0.1;
	drive.motionProfile.distanceMultiplier[1] = 0.9;
	drive.motionProfile.offsets[0] = 50;
	drive.motionProfile.offsets[1] = 0;
	drive.rectify = true
	for(int C = 0; drive.PID.notDone && C<1000; C++){
	DRIVE_forward(PID, 24, 127);
	delay(META_loopsDelay);
	}
	resetValues();
	writeDebugStreamLine("\nDone!!");
	//programmingSkillsAuton();
}
