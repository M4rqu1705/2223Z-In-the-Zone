/*   MobileGoalAndDriveTest.c - Program to test the Mobile Goal Intake      *
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
#include "commons.h"
#include "MobileGoalAndDriveTestFunctions.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	initialize();
}

task usercontrol(){
	//startTask(autonomous);
	while (true){
		driveOperatorControl();
		//armsOperatorControl();
		mobileGoalOperatorControl();
		delay(LOOPS_DELAY);
	}
}

task autonomous(){

	while(drive.notDone){
		move(TurnRight,90, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(drive.notDone){
		move(TurnLeft, 180, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned left");

	while(drive.notDone){
		move(TurnRight, 90, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Turned right");

	while(drive.notDone){
		move(Backward, 5, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved backward");

	while(drive.notDone){
		move(Forward, 10, 75);
		delay(1);
	}
	resetValues();
	delay(200);
	writeDebugStreamLine("Moved forward");

	while(drive.notDone){
		move(Backward, 5, 75);
		delay(1);
	}
	resetValues();
	writeDebugStreamLine("Done!!");
}
