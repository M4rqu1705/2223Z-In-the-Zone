/*   MobileGoalAndDriveTest2.c - Program to test the Mobile Goal Intake     *
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
#include "MobileGoalAndDriveTestFunctions2.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	initialize();
}

task usercontrol(){
	//startTask(autonomous);
	while (true){
		driveOperatorControl();
		mobileGoalOperatorControl(false);
		delay(LOOPS_DELAY);
	}
}

task autonomous(){
	resetValues();
	clearDebugStream();

	if(mobileGoalIntake.notDone){
		moveMobileGoal(0);
	}
	resetValues();
	writeDebugStreamLine("Extended Mobile Goal Intake");

	if(mobileGoalIntake.notDone){
		moveMobileGoal(1);
	}
	resetValues();
	writeDebugStreamLine("Retracted Mobile Goal Intake");
}
