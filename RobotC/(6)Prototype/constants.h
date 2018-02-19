/*   constants.h - File in which important constants are stored             *
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

#ifndef constants.h
#define constants.h

#pragma systemfile

//-Motor ports-------------------------------------------------------------------------------------------------------------------//
#define MOTOR_MOGO_L port3
#define MOTOR_DRIVE_LF port2
#define MOTOR_DRIVE_LB port1
#define MOTOR_ARM_L port4
#define MOTOR_CLAW_L port5
#define MOTOR_CLAW_R port6
#define MOTOR_ARM_R port7
#define MOTOR_DRIVE_RB port10
#define MOTOR_DRIVE_RF port9
#define MOTOR_MOGO_R port8

#define MOTOR_MOGO_L_TYPE tmotorVex393_HBridge
#define MOTOR_DRIVE_LF_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_DRIVE_LB_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_ARM_L_TYPE tmotorVex393_HBridge
#define MOTOR_CLAW_L_TYPE tmotorVex393_HBridge
#define MOTOR_CLAW_R_TYPE tmotorVex393_HBridge
#define MOTOR_ARM_R_TYPE tmotorVex393_HBridge
#define MOTOR_DRIVE_RB_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_DRIVE_RF_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_MOGO_R_TYPE tmotorVex393_HBridge

//-Sensor ports------------------------------------------------------------------------------------------------------------------//
#define POWER_EXPANDER_STATUS in1

#define SENSOR_POT_L in2
#define SENSOR_POT_R in3

#define SENSOR_GYRO in8

#define SENSOR_ENCODER_L dgtl1
#define SENSOR_ENCODER_R dgtl3
#define SENSOR_ENCODER_L_INVE	RTED false
#define SENSOR_ENCODER_R_INVERTED true

//-Joystick ports----------------------------------------------------------------------------------------------------------------//
#define JOYSTICK_DRIVE_F Ch2
#define JOYSTICK_DRIVE_S Ch4
#define JOYSTICK_DRIVE_INVERT Btn8U

#define JOYSTICK_ARM Btn5U
#define JOYSTICK_ARM_LOADER Btn5D

#define JOYSTICK_CLAWS Btn6U
#define JOYSTICK_MOGO Btn6D

//-PID---------------------------------------------------------------------------------------------------------------------------//
#define DRIVE_PID_KP_PRESET 0.75
#define DRIVE_PID_KI_PRESET 0.5
#define DRIVE_PID_KD_PRESET 5
#define DRIVE_PID_ERROR_PRESET 0
#define DRIVE_PID_INTEGRAL_PRESET 0
#define DRIVE_PID_INTEGRAL_LIMIT_PRESET 20000
#define DRIVE_PID_LAST_ERROR_PRESET 0

#define ARM_PID_KP_PRESET 0.75
#define ARM_PID_KI_PRESET 0.05
#define ARM_PID_KD_PRESET 3.0
#define ARM_PID_ERROR_PRESET 0
#define ARM_PID_INTEGRAL_PRESET 0
#define ARM_PID_INTEGRAL_LIMIT_PRESET 50
#define ARM_PID_LAST_ERROR_PRESET 0

#define PID_DONE_THRESHOLD 2

//-Others-------------------------------------------------------------------------------------------------------------------------//
#define LCD_BACKLIGHT true
//#define USING_LCD true	//Comment if false

#define DRIVE_THRESHOLD 15
#define DRIVE_PID_CORRECTION_CYCLES 10
#define SLEW_GAIN 9.7692307692307692307692307692308
// = 254/(floor(maxTime / DRIVERCONTROL_LOOP_DELAY))

#define DRIVE_WIDTH 13.0
#define WHEEL_DIAMETER 3.25
#define GYRO_FULL_ROTATION_TICKS 3600
#define RECTIFY_CONSTANT_ENCODER 1
#define RECTIFY_CONSTANT_GYRO 1

#define ARM_UP 1024
#define ARM_LOADER 512
#define ARM_DOWN 0

#define CLAW_SPEED 127
#define CLAWS_CYCLES 5

#define MOGO_CYCLES 100

#define POWER_EXPANDER_DIVISOR 45.6
//Use Divisors of 45.6 or 70
//https://www.vexrobotics.com/276-2271.html

#define DRIVERCONTROL_LOOP_DELAY 15
#endif
