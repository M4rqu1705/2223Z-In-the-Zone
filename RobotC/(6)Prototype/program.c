/*   program.c - Draft "main" file of the model 6 program prototype         *
*    Copyright (C) <2017>  Marcos Ricardo Pesante Colón                     *
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
#include "constants.h"
#include "functions.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	initialize();
}

task usercontrol(){
	//startTask(autonomous);
	while (true){
		driveOperatorControl();
		armsOperatorControl();
		clawsOperatorControl();
		mobileGoalOperatorControl();
		delay(DRIVERCONTROL_LOOP_DELAY);
	}
}

task autonomous(){

	while(!driveNotDone){
		drive(r,90, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(driveNotDone){
		drive(l,180, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned left");

	while(driveNotDone){
		drive(r,90, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(driveNotDone){
		drive(b,5, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved backward");

	while(driveNotDone){
		drive(f,10, 75, false);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved forward");

	while(driveNotDone){
		drive(b,5, 75, false);
		delay(1);
	}
	resetValues();
	writeDebugStreamLine("Done!!");
}
