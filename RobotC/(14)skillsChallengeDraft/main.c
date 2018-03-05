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
	/*//Turn right 90 degrees ready to deposit mobile goal
	resetValues();
	SensorValue[SENSOR_gyro] = 0;
	LOADED_mobileGoal(false, false, true, false);
	for(int C = 0; drive.PID.notDone && C<45; C++){
		DRIVE_turnRight(Gyro, 90, 6.5, 127);
		ARM_move(PID, 1);
		delay(META_loopsDelay);
	}*/
	programmingSkillsAuton();
}
