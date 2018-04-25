/*   main.c - Main file containing program for Daniela's mini robot         *
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
#include "LCD.h"

void pre_auton(){
	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = true;
	LCD_init();
	initialize();

	while(bIfiRobotDisabled){
		LCD_select();
	}
}

task autonomous(){

	mobileGoalIsLoaded(false, false, false, false);
	pidInit(lift.PID, PID_KPlift[0], PID_KIlift[0], PID_KDlift[0], 0, 0, 0, 0, 0, PID_correctionCyclesLift);
	pidInit(coneIntake.miniFourbar.PID, PID_KPminiFourbar[1], PID_KIminiFourbar[0], PID_KDminiFourbar[0], 0, 0, 0, 0, 0, PID_correctionCyclesMiniFourbar);
	for(int C = 0; C<100; C++){
		moveDrive(47, 127);
		moveMobileGoalIntake(META_mobileGoalIntakeExtended);
		moveLift(800, 127);
		moveMiniFourbar(META_miniFourbarUp);
		moveGoliathIntake(-30);
		delay(META_loopsDelay);
	}
	resetValues();

	mobileGoalIsLoaded(false, false, false, false);
	pidInit(lift.PID, PID_KPlift[0], PID_KIlift[0], PID_KDlift[0], 0, 0, 0, 0, 0, PID_correctionCyclesLift);
	pidInit(coneIntake.miniFourbar.PID, PID_KPminiFourbar[1], PID_KIminiFourbar[0], PID_KDminiFourbar[0], 0, 0, 0, 0, 0, PID_correctionCyclesMiniFourbar);
	for(int C = 0; C<25; C++){
		moveMobileGoalIntake(META_mobileGoalIntakeRetracted);
		moveLift(700, 127);
		moveMiniFourbar(META_miniFourbarUp);
		moveGoliathIntake(-50);
		delay(META_loopsDelay);
	}
	resetValues();

	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	mobileGoalIsLoaded(false, false, false, false);
	pidInit(lift.PID, PID_KPlift[0], PID_KIlift[0], PID_KDlift[0], 0, 0, 0, 0, 0, PID_correctionCyclesLift);
	pidInit(coneIntake.miniFourbar.PID, PID_KPminiFourbar[1], PID_KIminiFourbar[0], PID_KDminiFourbar[0], 0, 0, 0, 0, 0, PID_correctionCyclesMiniFourbar);
	for(int C = 0; C<100; C++){
		moveDrive(-45, 127);
		moveMobileGoalIntake(META_mobileGoalIntakeRetracted);
		moveLift(200, 127);
		moveMiniFourbar(META_miniFourbarUp);
		moveGoliathIntake(-50);
		delay(META_loopsDelay);
	}
	resetValues();

	mobileGoalIsLoaded(false, false, true, false);
	pidInit(coneIntake.miniFourbar.PID, PID_KPminiFourbar[1], PID_KIminiFourbar[0], PID_KDminiFourbar[0], 0, 0, 0, 0, 0, PID_correctionCyclesMiniFourbar);
	for(int C = 0; C<75; C++){
		turnDrive(135, 127);
		moveLift(200, 127);
		moveMiniFourbar(META_miniFourbarUp);
		moveGoliathIntake(50);
		delay(META_loopsDelay);
	}
	resetValues();


	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	mobileGoalIsLoaded(false, false, false, false);
	pidInit(lift.PID, PID_KPlift[0], PID_KIlift[0], PID_KDlift[0], 0, 0, 0, 0, 0, PID_correctionCyclesLift);
	pidInit(coneIntake.miniFourbar.PID, PID_KPminiFourbar[1], PID_KIminiFourbar[0], PID_KDminiFourbar[0], 0, 0, 0, 0, 0, PID_correctionCyclesMiniFourbar);
	for(int C = 0; C<90; C++){
		moveDrive(24, 127);
		moveMobileGoalIntake(META_mobileGoalIntakeRetracted);
		moveLift(500, 127);
		delay(META_loopsDelay);
	}
	resetValues();


	mobileGoalIsLoaded(false, false, true, false);
	pidInit(coneIntake.miniFourbar.PID, PID_KPminiFourbar[1], PID_KIminiFourbar[1], PID_KDminiFourbar[1], 0, 0, 0, 0, 0, PID_correctionCyclesMiniFourbar);
	for(int C = 0; C<60; C++){
		turnDrive(230, 127);
		moveLift(500, 127);
		delay(META_loopsDelay);
	}
	resetValues();

	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	mobileGoalIsLoaded(false, false, false, false);
	pidInit(lift.PID, PID_KPlift[0], PID_KIlift[0], PID_KDlift[0], 0, 0, 0, 0, 0, PID_correctionCyclesLift);
	pidInit(coneIntake.miniFourbar.PID, PID_KPminiFourbar[1], PID_KIminiFourbar[0], PID_KDminiFourbar[0], 0, 0, 0, 0, 0, PID_correctionCyclesMiniFourbar);
	for(int C = 0; C<90; C++){
		moveDrive(35, 127);
		moveMobileGoalIntake(META_mobileGoalIntakeRetracted);
		moveLift(500, 127);
		delay(META_loopsDelay);
	}
	resetValues();

	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	mobileGoalIsLoaded(false, false, false, false);
	pidInit(lift.PID, PID_KPlift[0], PID_KIlift[0], PID_KDlift[0], 0, 0, 0, 0, 0, PID_correctionCyclesLift);
	pidInit(coneIntake.miniFourbar.PID, PID_KPminiFourbar[1], PID_KIminiFourbar[0], PID_KDminiFourbar[0], 0, 0, 0, 0, 0, PID_correctionCyclesMiniFourbar);
	for(int C = 0; C<15; C++){
		moveMobileGoalIntake(META_mobileGoalIntakeExtended);
		moveLift(500, 127);
		delay(META_loopsDelay);
	}
	resetValues();

	SensorValue[SENSOR_encoderL] = SensorValue[SENSOR_encoderR] = 0;
	mobileGoalIsLoaded(false, false, false, false);
	pidInit(lift.PID, PID_KPlift[0], PID_KIlift[0], PID_KDlift[0], 0, 0, 0, 0, 0, PID_correctionCyclesLift);
	pidInit(coneIntake.miniFourbar.PID, PID_KPminiFourbar[1], PID_KIminiFourbar[0], PID_KDminiFourbar[0], 0, 0, 0, 0, 0, PID_correctionCyclesMiniFourbar);
	for(int C = 0; C<50; C++){
		moveDrive(-40, 127);
		if(C<10)moveMobileGoalIntake(META_mobileGoalIntakeExtended);
		moveLift(500, 127);
		delay(META_loopsDelay);
	}
	resetValues();
}

task usercontrol(){
	resetValues();
	while(true){
		operatorControl();
		delay(META_loopsDelay);
	}
}
