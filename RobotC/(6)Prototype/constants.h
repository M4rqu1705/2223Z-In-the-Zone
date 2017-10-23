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
#define SENSOR_ENCODER_L_INVERTED false
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
#define DRIVE_PID_KP_PRESET 1.7
#define DRIVE_PID_KI_PRESET 1
#define DRIVE_PID_KD_PRESET 2
#define DRIVE_PID_ERROR_PRESET 0
#define DRIVE_PID_INTEGRAL_PRESET 0
#define DRIVE_PID_INTEGRAL_LIMIT_PRESET 1000
#define DRIVE_PID_LAST_ERROR_PRESET 0

#define ARM_PID_KP_PRESET 0.75
#define ARM_PID_KI_PRESET 0.05
#define ARM_PID_KD_PRESET 3.0
#define ARM_PID_ERROR_PRESET 0
#define ARM_PID_INTEGRAL_PRESET 0
#define ARM_PID_INTEGRAL_LIMIT_PRESET 50
#define ARM_PID_LAST_ERROR_PRESET 0

#define PID_DONE_THRESHOLD 5

//-Kalman filter-----------------------------------------------------------------------------------------------------------------//
#define DRIVE_FILTERS_KG_PRESET 0.5
#define DRIVE_FILTERS_ESTIMATE_PRESET 0
#define DRIVE_FILTERS_PREVIOUS_ESTIMATE_PRESET 0
#define DRIVE_FILTERS_ERROR_ESTIMATE_PRESET 5
#define DRIVE_FILTERS_PREVIOUS_ERROR_ESTIMATE_PRESET 5
#define DRIVE_FILTERS_ERROR_MEASUREMENT_PRESET 5

#define ARM_FILTERS_KG_PRESET 0.5
#define ARM_FILTERS_ESTIMATE_PRESET 0
#define ARM_FILTERS_PREVIOUS_ESTIMATE_PRESET 0
#define ARM_FILTERS_ERROR_ESTIMATE_PRESET 5
#define ARM_FILTERS_PREVIOUS_ERROR_ESTIMATE_PRESET 5
#define ARM_FILTERS_ERROR_MEASUREMENT_PRESET 5

//#define USING_KALMAN_FILTER true //Comment if false

//-Others-------------------------------------------------------------------------------------------------------------------------//
#define LCD_BACKLIGHT true
//#define USING_LCD true	//Comment if false

#define DRIVE_THRESHOLD 15
#define DRIVE_PID_CORRECTION_CYCLES 10
#define SLEW_GAIN 12.7
// = 254/(floor(maxTime) / DRIVERCONTROL_LOOP_DELAY)

#define DRIVE_WIDTH 13.0
#define WHEEL_DIAMETER 3.25
#define GYRO_FULL_ROTATION_TICKS 3600
#define RECTIFY_CONSTANT_ENCODER 0.02
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

/*


#pragma config(Sensor, in1,    powerExpanderStatus, sensorAnalog)
#pragma config(Sensor, in2,    potL,           sensorPotentiometer)
#pragma config(Sensor, in3,    potR,           sensorPotentiometer)
#pragma config(Sensor, in8,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  encoderL,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  encoderR,       sensorQuadEncoder)
#pragma config(Motor,  port1,           driveLB,       tmotorVex393_HBridge, openLoop, driveLeft, encoderPort, dgtl1)
#pragma config(Motor,  port2,           driveLF,       tmotorVex393HighSpeed_MC29, openLoop, driveLeft, encoderPort, dgtl1)
#pragma config(Motor,  port3,           mogoL,         tmotorVex393HighSpeed_MC29, openLoop)s
#pragma config(Motor,  port4,           armL,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           clawL,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           clawR,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           armR,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           mogoR,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           driveRF,       tmotorVex393HighSpeed_MC29, openLoop, reversed, driveRight, encoderPort, dgtl3)
#pragma config(Motor,  port10,          driveRB,       tmotorVex393_HBridge, openLoop, reversed, driveRight, encoderPort, dgtl3)
*/


#endif
