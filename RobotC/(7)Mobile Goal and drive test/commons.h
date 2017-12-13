#ifndef commons.h
#define commons.h

#pragma systemfile

//-Motor ports-------------------------------------------------------------------------------------------------------------------//
#define MOTOR_DRIVE_LB port1
#define MOTOR_DRIVE_LF port2
#define MOTOR_CLAWS port3
#define MOTOR_ARM_LO port4
#define MOTOR_ARM_LI port5
#define MOTOR_ARM_RI port6
#define MOTOR_ARM_RO port7
#define MOTOR_MOBILE_GOAL port8
#define MOTOR_DRIVE_RF port9
#define MOTOR_DRIVE_RB port10

#define MOTOR_DRIVE_LB_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_DRIVE_LF_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_CLAWS_TYPE tmotorVex393_HBridge
#define MOTOR_ARM_LO_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_ARM_LI_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_ARM_RI_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_ARM_RO_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_MOBILE_GOAL_TYPE tmotorVex393_HBridge
#define MOTOR_DRIVE_RF_TYPE tmotorVex393HighSpeed_MC29
#define MOTOR_DRIVE_RB_TYPE tmotorVex393HighSpeed_MC29

//-Sensor ports------------------------------------------------------------------------------------------------------------------//
#define POWER_EXPANDER_STATUS in1

#define SENSOR_POT_MOGO in2
#define SENSOR_POT_L in3
#define SENSOR_POT_R in4

#define SENSOR_ENCODER_L dgtl1
#define SENSOR_ENCODER_R dgtl3
#define SENSOR_ENCODER_L_INVERTED false
#define SENSOR_ENCODER_R_INVERTED true

//-Joystick ports----------------------------------------------------------------------------------------------------------------//
#define JOYSTICK_DRIVE_F Ch3
#define JOYSTICK_DRIVE_S Ch1
#define JOYSTICK_DRIVE_INVERT Btn8R

#define JOYSTICK_ARM_UP Btn5U
#define JOYSTICK_ARM_DOWN Btn5D

#define JOYSTICK_MOBILE_GOAL_INTAKE_EXTEND Btn6U
#define JOYSTICK_MOBILE_GOAL_INTAKE_RETRACT Btn6D

#define DRIVE_THRESHOLD 15

//-PID---------------------------------------------------------------------------------------------------------------------------//
#define PID_DRIVE_KP_PRESET 0.75
#define PID_DRIVE_KI_PRESET 0.5
#define PID_DRIVE_KD_PRESET 5
#define PID_DRIVE_INTEGRAL_MAX_PRESET 127
#define PID_DRIVE_ERROR_PRESET 0
#define PID_DRIVE_LAST_ERROR_PRESET 0
#define PID_DRIVE_INTEGRAL_PRESET 0

#define PID_ARMS_KP_PRESET 0.25
#define PID_ARMS_KI_PRESET 1.0
#define PID_ARMS_KD_PRESET 20
#define PID_ARMS_INTEGRAL_MAX_PRESET 127
#define PID_ARMS_ERROR_PRESET 0
#define PID_ARMS_LAST_ERROR_PRESET 0
#define PID_ARMS_INTEGRAL_PRESET 0

#define PID_MOBILE_GOAL_KP_PRESET 1
#define PID_MOBILE_GOAL_KI_PRESET 1
#define PID_MOBILE_GOAL_KD_PRESET 10
#define PID_MOBILE_GOAL_INTEGRAL_MAX_PRESET 127
#define PID_MOBILE_GOAL_ERROR_PRESET 0
#define PID_MOBILE_GOAL_LAST_ERROR_PRESET 0
#define PID_MOBILE_GOAL_INTEGRAL_PRESET 0

#define PID_DRIVE_CORRECTION_CYCLES 10
#define PID_ARMS_CORRECTION_CYCLES 10
#define PID_MOBILE_GOAL_CORRECTION_CYCLES 10

#define PID_DONE_THRESHOLD 2

//-Others-------------------------------------------------------------------------------------------------------------------------//
#define LCD_BACKLIGHT true
//#define USING_LCD true	//Comment if false

#define MOBILE_GOAL_EXTENDED_INTAKE 2450
#define MOBILE_GOAL_RETRACTED_INTAKE 950
#define ARM_LEFT_DOWN 4094
#define ARM_LEFT_UP 1000
#define ARM_RIGHT_DOWN 1
#define ARM_RIGHT_UP 3000


#define SLEW_GAIN 25.4
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

#define POWER_EXPANDER_DIVISOR 45.6
//Use Divisors of 45.6 or 70
//https://www.vexrobotics.com/276-2271.html

#define LOOPS_DELAY 20
#endif
