/*   commons.h - Necessary constants for program operation                  *
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

#ifndef COMMONS_H_
#define COMMONS_H_

#pragma systemfile

//Motors and sensors setup
#define MOTOR_driveLF port4
#define MOTOR_driveLM port9
#define MOTOR_driveLB port10

#define MOTOR_mobileGoalL port5
#define MOTOR_coneIntake port3
#define MOTOR_arm port8
#define MOTOR_mobileGoalR port6

#define MOTOR_driveRB port1
#define MOTOR_driveRM port2
#define MOTOR_driveRF port7

#define SENSOR_potArm in1
#define SENSOR_potMogo in2
#define SENSOR_powerExpander in3
#define SENSOR_encoderL dgtl1
#define SENSOR_encoderR dgtl3
#define SENSOR_gyro in8

//Joystick setup
#define JOYSTICK_driveF Ch3
#define JOYSTICK_driveS Ch4
#define JOYSTICK_arm Ch2
#define JOYSTICK_mobileGoalE Btn6D
#define JOYSTICK_mobileGoalR Btn6U
#define JOYSTICK_coneIntakeP Btn5U
#define JOYSTICK_coneIntakeD Btn5D

//PID variables setup
//Drive without Mobile Goal
float PID_KPdriveUnloaded = 0.3;
float PID_KIdriveUnloaded = 0.3;
float PID_KDdriveUnloaded = 0.125;
#define PID_integralMaxDrive 127
#define PID_correctionCyclesDriveUnloaded 20
#define PID_correctionThresholdDriveUnloaded 5
unsigned byte PID_timeoutDriveUnloaded = 100;

//Drive with Mobile Goal
float PID_KPdriveLoaded = 0.45;
float PID_KIdriveLoaded = 0.45;
float PID_KDdriveLoaded = 0.15;
#define PID_correctionCyclesDriveLoaded 20
#define PID_correctionThresholdDriveLoaded 15
unsigned byte PID_timeoutDriveLoaded = 100;

//Drive with Gyro
float PID_KPdriveGyroLoaded = 0.4;
float PID_KIdriveGyroLoaded = 0.5;
float PID_KDdriveGyroLoaded = 0.05;

float PID_KPdriveGyroUnloaded = 0.4;
float PID_KIdriveGyroUnloaded = 0.45;
float PID_KDdriveGyroUnloaded = 0.05;

//Mobile Goal intake retracts without Mobile Goal
float PID_KPmobileGoalIntakeUnloadedRetract = 0.15;
float PID_KImobileGoalIntakeUnloadedRetract = 0;
float PID_KDmobileGoalIntakeUnloadedRetract = 0;
#define PID_integralMaxMobileGoalIntake 127
#define PID_correctionCyclesMobileGoalIntakeUnloadedRetract 50
#define PID_correctionThresholdMobileGoalIntakeUnloadedRetract 10
unsigned byte PID_timeoutMobileGoalIntakeUnloadedRetract = 100;

//Mobile Goal Intake retracts with Mobile Goal
float PID_KPmobileGoalIntakeLoadedRetract = 0.3;
float PID_KImobileGoalIntakeLoadedRetract = 0.3;
float PID_KDmobileGoalIntakeLoadedRetract = 0.025;
#define PID_correctionCyclesMobileGoalIntakeLoadedRetract 50
#define PID_correctionThresholdMobileGoalIntakeLoadedRetract 10
unsigned byte PID_timeoutMobileGoalIntakeLoadedRetract = 100;

//Mobile Goal intake extends without Mobile Goal
float PID_KPmobileGoalIntakeUnloadedExtend = 0.15;
float PID_KImobileGoalIntakeUnloadedExtend = 0.15;
float PID_KDmobileGoalIntakeUnloadedExtend = 0.025;
#define PID_correctionCyclesMobileGoalIntakeUnloadedExtend 50
#define PID_correctionThresholdMobileGoalIntakeUnloadedExtend 10
unsigned byte PID_timeoutMobileGoalIntakeUnloadedExtend = 100;

//Mobile Goal Intake extends with Mobile Goal
float PID_KPmobileGoalIntakeLoadedExtend = 0.25;
float PID_KImobileGoalIntakeLoadedExtend = 0.1;
float PID_KDmobileGoalIntakeLoadedExtend = 0.025;
#define PID_correctionCyclesMobileGoalIntakeLoadedExtend 50
#define PID_correctionThresholdMobileGoalIntakeLoadedExtend 10
unsigned byte PID_timeoutMobileGoalIntakeLoadedExtend = 100;

//Arm rises with cone
float PID_KParmUp = 0.5;
float PID_KIarmUp = 0.4;
float PID_KDarmUp = 0.05;
#define PID_integralMaxArm 127
#define PID_correctionCyclesArmUp 50
#define PID_correctionThresholdArmUp 10
unsigned byte PID_timeoutArmUp = 100;

//Arm lowers without cone
float PID_KParmDown = 0.4;
float PID_KIarmDown = 0.4;
float PID_KDarmDown = 0.05;
#define PID_correctionCyclesArmDown 50
#define PID_correctionThresholdArmDown 10
unsigned byte PID_timeoutArmDown = 100;

//Information needed for calculations
#define META_driveOpControlThreshold 20
#define META_driveWidth 13.0
#define META_driveWheelDiameter 3.25
#define META_encoderLInverted false
#define META_encoderRInverted true
#define META_slewGain 12.7
// = 254/(floor(maxTime / DRIVERCONTROL_LOOP_DELAY))
#define META_slewGainThreshold 10
#define PI 3.141592653589793

#define META_mogoOpControlThreshold 40
#define META_mogoExtended 700
#define META_mogoRetracted 2400
#define META_mogoMaxOutput 127

#define META_armOpControlThreshold 50
#define META_armUp 2900
#define META_armDown 1250
#define META_armScore 2324
#define META_armLoader 1990
#define META_armMaxOutput 127

#define META_coneIntakeSpeed 100
#define META_coneIntakeCycles 100
float META_coneIntakeMaxOutput = 40;

#define META_lcdBacklight true
#define META_usingLCD //comment if will not use LCD

#define META_powerExpanderInputDivisor 70
//Use divisors of 45.6 or

#define META_loopsDelay 20

#define META_scaleFactor 1

#endif
