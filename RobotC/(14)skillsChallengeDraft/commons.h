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
#define JOYSTICK_driveS Ch1
#define JOYSTICK_arm Ch2
#define JOYSTICK_mobileGoalE Btn6D
#define JOYSTICK_mobileGoalR Btn6U
#define JOYSTICK_coneIntakeP Btn5U
#define JOYSTICK_coneIntakeD Btn5D

//PID variables setup
//Format:[Loaded][speedPID] or {{unloadedPosition PID, unloadedSpeed PID}, {loaded Position PID, loaded speed PID}}
float PID_KPdrive[2][2] = {{1,0.75}, {0.45,0.75}};
float PID_KIdrive[2][2] = {{0.3,0.5}, {0.45,0.5}};
float PID_KDdrive[2][2] = {{0.0125,0.005}, {0.15,0.005}};
float PID_KPdriveGyro[2][2] = {{0.3,1}, {0.3,1}};
float PID_KIdriveGyro[2][2] = {{0.3,0}, {0.3,0.5}};
float PID_KDdriveGyro[2][2] = {{0.04,0}, {0.04,0.005}};
#define PID_integralMaxDrive 127
#define PID_correctionCyclesDriveUnloaded 20
#define PID_correctionThresholdDriveUnloaded 5

//Format: [Loaded][Retract]
float PID_KPmobileGoalIntake[2][2] = {{0.15,0.15}, {0.25,0.3}};
float PID_KImobileGoalIntake[2][2] = {{0.15,0}, {0.1,0.3}};
float PID_KDmobileGoalIntake[2][2] = {{0.025,0}, {0.025,0.025}};
#define PID_integralMaxMobileGoalIntake 127
#define PID_correctionCyclesMobileGoalIntake 50
#define PID_correctionThresholdMobileGoalIntake 10

//Format: [Down]
float PID_KParm[2] = {0.5, 0.4};
float PID_KIarm[2] = {0.4, 0.4};
float PID_KDarm[2] = {0.005, 0.05};
#define PID_integralMaxArm 127
#define PID_correctionCyclesArm 50
#define PID_correctionThresholdArm 10

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
#define META_mogoExtended 735
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

#define META_LCDbacklight true
#define META_usingLCD //comment if will not use LCD

#define META_powerExpanderInputDivisor 70
//Use divisors of 45.6 or

#define META_loopsDelay 20

#define META_scaleFactor 1

#endif
