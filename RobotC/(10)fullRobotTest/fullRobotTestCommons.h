/*   fullRobotTestCommons.h - Useful constants for other parts of the program *
*    Copyright (C) <2018>  Marcos Ricardo Pesante Colón                       *
*                                                                             *
*    This program is free software: you can redistribute it and/or modify     *
*    it under the terms of the GNU General Public License as published by     *
*    the Free Software Foundation, either version 3 of the License, or        *
*    (at your option) any later version.                                      *
*                                                                             *
*    This program is distributed in the hope that it will be useful,          *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
*    GNU General Public License for more details.                             *
*                                                                             *
*    You should have received a copy of the GNU General Public License        *
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.    */

#ifndef fullRobotTestCommons.h
#define fullRobotTestCommons.h

#pragma systemfile

//-Motor ports-------------------------------------------------------------------------------------------------------------------//
//L = Left, R = Right, F = Front, M = Middle, B = Back
#define MOTOR_driveLB port1
#define MOTOR_claw port2
#define MOTOR_arm port3
#define MOTOR_driveLF port4
#define MOTOR_driveLM port5
#define MOTOR_driveRM port6
#define MOTOR_driveRF port7
#define MOTOR_mogoIntake port9
#define MOTOR_driveRB port10

#define MOTORTYPE_driveLB tmotorVex393TurboSpeed_HBridge
#define MOTORTYPE_claw tmotorVex393_HBridge
#define MOTORTYPE_arm tmotorVex393HighSpeed_MC29
#define MOTORTYPE_driveLF tmotorVex393TurboSpeed_HBridge
#define MOTORTYPE_driveLM tmotorVex393TurboSpeed_HBridge
#define MOTORTYPE_driveRM tmotorVex393TurboSpeed_HBridge
#define MOTORTYPE_driveRF tmotorVex393TurboSpeed_HBridge
#define MOTORTYPE_mogoIntake tmotorVex393_HBridge
#define MOTORTYPE_driveRB tmotorVex393TurboSpeed_HBridge

#define MOTORINVERT_driveLB false
#define MOTORINVERT_claw false
#define MOTORINVERT_arm false
#define MOTORINVERT_driveLF false
#define MOTORINVERT_driveLM false
#define MOTORINVERT_driveRM true
#define MOTORINVERT_driveRF true
#define MOTORINVERT_mogoIntake false
#define MOTORINVERT_driveRB true

//Sensor ports----Sensor ports----Sensor ports----Sensor ports----Sensor ports----Sensor ports----Sensor ports----Sensor ports//
#define SENSOR_powerExpanderStatus in1

#define SENSOR_potMogo in2
#define SENSOR_potArm in3

#define SENSOR_encoderL dgtl1
#define SENSOR_encoderR dgtl3

//Joystick ports----Joystick ports----Joystick ports----Joystick ports----Joystick ports----Joystick ports----Joystick ports//
#define JOYSTICK_driveF Ch3
#define JOYSTICK_driveS Ch4
#define JOYSTICK_driveInvert Btn7D

#define JOYSTICK_armUp Btn5U
#define JOYSTICK_armDown Btn5D
#define JOYSTICK_armPID Btn8D
#define JOYSTICK_armAnalog Ch2

#define JOYSTICK_mogoExtend Btn6D
#define JOYSTICK_mogoRetract Btn6U

#define JOYSTICK_clawOpen JOYSTICK_armUp
#define JOYSTICK_clawClose JOYSTICK_armDown

/*PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID----PID//
KP = P constant, KI = I constant, KD = D constant, integralMax = limit for the integral so it doesn't overflow,
doneThreshold = threshold to give flexibility when determining "ifDone", correctionCycles = how much time to wait to determine "ifDone"
*/
double PID_KPdrive = 0.75;
double PID_KIdrive = 0.5;
double PID_KDdrive =95;
#define PID_integralMaxDrive 127
#define PID_doneThresholdDrive 2
#define PID_correctionCyclesDrive 10

double PID_KParm = 0.2;
double PID_KIarm = 0.05;
double PID_KDarm = 0.3;
#define PID_integralMaxArm 127
#define PID_doneThresholdArm 2
#define PID_correctionCyclesArm 10

double PID_KPmogoExtend = 0.25;
double PID_KImogoExtend = 0.0;
double PID_KDmogoExtend = 3.0;
#define PID_integralMaxMogoExtend 127
#define PID_doneThresholdMogoExtend 5
#define PID_correctionCyclesMogoExtend 10

double PID_KPmogoRetract = 1;
double PID_KImogoRetract = 0.5;
double PID_KDmogoRetract = 4.0;
#define PID_integralMaxMogoRetract 127
#define PID_doneThresholdMogoRetract 5
#define PID_correctionCyclesMogoRetract 10

//Information----Information----Information----Information----Information----Information----Information----Information//

#define META_driveOpControlThreshold 20
#define META_driveWidth 13.0
#define META_driveWheelDiameter 3.25
#define META_driveEncoderRectifyConstant 1
#define META_encoderLInverted false
#define META_encoderRInverted true
#define META_slewGain 16.933333333333333333333333333333
// = 254/(floor(maxTime / DRIVERCONTROL_LOOP_DELAY))
#define META_slewGainThreshold 2

#define META_mogoExtended 2710
#define META_mogoRetracted 1050
#define META_mogoMaxOutput 127

#define META_armOpControlThreshold 50
#define META_armUp 2375
#define META_armDown 0

#define META_clawSpeed 40
#define META_clawCycles 10

#define META_lcdBacklight true
//#define META_usingLCD true //Comment if false

#define META_powerExpanderInputDivisor 45.6
//Use Divisors of 45.6 or 70
//https://www.vexrobotics.com/276-2271.html

#define META_usingLCD true
#define META_LCDbacklight true

bool META_armNotUsePID = false;

#define LOOPS_DELAY 20
#endif
