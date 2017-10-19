#ifndef constants.h
#define constants.h

#pragma systemfile

/*//-Motor ports-------------------------------------------------------------------------------------------------------------------//
#define MOTOR_MOGO_L port1
#define MOTOR_DRIVE_LF port2
#define MOTOR_DRIVE_LB port3
#define MOTOR_ARM_L port4
#define MOTOR_CLAW_L port5
#define MOTOR_CLAW_R port6
#define MOTOR_ARM_R port7
#define MOTOR_DRIVE_RB port8
#define MOTOR_DRIVE_RF port9
#define MOTOR_MOGO_R port10

//#define SENSOR_GYRO in8

//#define SENSOR_POT_L in2
//#define SENSOR_POT_R in3

//#define SENSOR_ENCODER_L dgtl1
//#define SENSOR_ENCODER_R dgtl3

#pragma config(Sensor, SENSOR_POT_L,    potL,           sensorPotentiometer)
#pragma config(Sensor, SENSOR_POT_R,    potR,           sensorPotentiometer)
#pragma config(Sensor, SENSOR_GYRO,    gyro,           sensorGyro)
#pragma config(Sensor, SENSOR_ENCODER_L,  encoderL,       sensorQuadEncoder)
#pragma config(Sensor, SENSOR_ENCODER_R,  encoderR,       sensorQuadEncoder)
#pragma config(Motor,  MOTOR_MOGO_L,           mogoL,         tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  MOTOR_DRIVE_LF,           driveLF,       tmotorVex393HighSpeed_MC29, openLoop, encoderPort, dgtl1)
#pragma config(Motor,  MOTOR_DRIVE_LB,           driveLB,       tmotorVex393HighSpeed_MC29, openLoop, encoderPort, dgtl1)
#pragma config(Motor,  MOTOR_ARM_L,           armL,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  MOTOR_CLAW_L,           clawL,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  MOTOR_CLAW_R,           clawR,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  MOTOR_ARM_R,           armR,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  MOTOR_DRIVE_RB,           driveRB,       tmotorVex393HighSpeed_MC29, openLoop, encoderPort, dgtl3)
#pragma config(Motor,  MOTOR_DRIVE_RF,           driveRF,       tmotorVex393HighSpeed_MC29, openLoop, encoderPort, dgtl3)
#pragma config(Motor,  MOTOR_MOGO_R,          mogoR,         tmotorVex393_HBridge, openLoop)
*/

//-Sensor ports------------------------------------------------------------------------------------------------------------------//
#define SENSOR_ENCODER_L_INVERTED false
#define SENSOR_ENCODER_R_INVERTED true


//-Joystick ports----------------------------------------------------------------------------------------------------------------//
#define JOYSTICK_DRIVE_F Ch3
#define JOYSTICK_DRIVE_S Ch1
#define JOYSTICK_DRIVE_INVERT Btn8U

#define JOYSTICK_ARM Btn5U
#define JOYSTICK_ARM_LOADER Btn5D

#define JOYSTICK_CLAWS Btn6U
#define JOYSTICK_MOGO Btn6D

//-PID---------------------------------------------------------------------------------------------------------------------------//
#define DRIVE_PID_KP_PRESET 1.5
#define DRIVE_PID_KI_PRESET 0.05
#define DRIVE_PID_KD_PRESET 2.0
#define DRIVE_PID_ERROR_PRESET 0
#define DRIVE_PID_INTEGRAL_PRESET 0
#define DRIVE_PID_INTEGRAL_LIMIT_PRESET 35
#define DRIVE_PID_LAST_ERROR_PRESET 0

#define ARM_PID_KP_PRESET .75
#define ARM_PID_KI_PRESET 0.05
#define ARM_PID_KD_PRESET 3.0
#define ARM_PID_ERROR_PRESET 0
#define ARM_PID_INTEGRAL_PRESET 0
#define ARM_PID_INTEGRAL_LIMIT_PRESET 20
#define ARM_PID_LAST_ERROR_PRESET 0

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
#define USING_LCD true	//Comment if false

#define DRIVE_THRESHOLD 15
#define SLEW_GAIN 12.7
// = 254/(floor(maxTime) / DRIVERCONTROL_LOOP_DELAY)

#define DRIVE_WIDTH 17.5
#define WHEEL_DIAMETER 3.25
#define GYRO_FULL_ROTATION_TICKS 4000
#define PID_DONE_THRESHOLD 10

#define ARM_UP 1024
#define ARM_LOADER 512
#define ARM_DOWN 0

#define CLAW_SPEED 127
#define CLAWS_CYCLES 5

#define MOGO_CYCLES 100

#define DRIVERCONTROL_LOOP_DELAY 15

#define PI 3.141592653589793

#endif
