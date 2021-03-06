/*   fullRobotTest.c - Main file and program                                *
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

#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"
#include "fullRobotTestCommons.h"
#include "fullRobotTestFunctions.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;
	initialize();
}

task usercontrol(){
	//startTask(autonomous);
	while (true){
		driveOperatorControl(false);
		armOperatorControl(false, META_armNotUsePID, true);
		mobileGoalOperatorControl(false, false);
		lcdCalibrate();
		delay(LOOPS_DELAY);
	}
}

task autonomous(){
	resetValues();
	clearDebugStream();

	while(claw.notDone && arm.notDone){
		moveClaw(close, 50);
		moveArm(1);
	}
	resetValues();

	while(drive.notDone && mobileGoalIntake.notDone && arm.notDone){
		move(Forward, 12, 75);
		moveMobileGoal(0);
		moveArm(0);
	}
	resetValues();

	while(drive.notDone){
		move(Forward, 12, 50);
	}
	resetValues();

	while(mobileGoalIntake.notDone){
		moveMobileGoal(1);
	}
	resetValues();

	while(drive.notDone){
		move(TurnRight, 180, 50);
	}
	resetValues();

	while(drive.notDone){
		move(Forward, 5, 75);
	}
	resetValues();

	while(drive.notDone && mobileGoalIntake.notDone){
		move(Backward, 5, 50);
		moveMobileGoal(0);
	}
	resetValues();
}
