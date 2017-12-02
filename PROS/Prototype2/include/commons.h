#ifndef COMMONS_H_INCLUDED_
#define COMMONS_H_INCLUDED_

//-Motor ports-------------------------------------------------------------------------------------------------------------------//
#define MOTOR_MOGO_L 3
#define MOTOR_DRIVE_LF 2
#define MOTOR_DRIVE_LB 1
#define MOTOR_ARM_L 4
#define MOTOR_CLAW_L 5
#define MOTOR_CLAW_R 6
#define MOTOR_ARM_R 7
#define MOTOR_DRIVE_RB 10
#define MOTOR_DRIVE_RF 9
#define MOTOR_MOGO_R 8

//-Sensor ports------------------------------------------------------------------------------------------------------------------//
#define POWER_EXPANDER_STATUS 1

#define SENSOR_POT_L 2
#define SENSOR_POT_R 3

#define SENSOR_GYRO 8

#define SENSOR_ENCODER_L 1
#define SENSOR_ENCODER_R 3
#define SENSOR_ENCODER_L_INVERTED false
#define SENSOR_ENCODER_R_INVERTED true

//-Joystick ports----------------------------------------------------------------------------------------------------------------//
#define JOYSTICK_DRIVE_F 2
#define JOYSTICK_DRIVE_S 4
#define JOYSTICK_DRIVE_INVERT 8
#define JOYSTICK_DRIVE_INVERT_BUTTON JOY_UP

#define JOYSTICK_ARM 5
#define JOYSTICK_ARM_BUTTON JOY_UP
#define JOYSTICK_ARM_LOADER 5
#define JOYSTICK_ARM_LOADER_BUTTON JOY_DOWN

#define JOYSTICK_CLAWS 6
#define JOYSTICK_CLAWS_BUTTON JOY_UP

#define JOYSTICK_MOGO 6
#define JOYSTICK_MOGO_BUTTON JOY_DOWN

//-PID---------------------------------------------------------------------------------------------------------------------------//
#define DRIVE_PID_KP_PRESET 0.75
#define DRIVE_PID_KI_PRESET 0.5
#define DRIVE_PID_KD_PRESET 5
#define DRIVE_PID_INTEGRAL_LIMIT_PRESET 20000

#define ARM_PID_KP_PRESET 0.75
#define ARM_PID_KI_PRESET 0.05
#define ARM_PID_KD_PRESET 3.0
#define ARM_PID_INTEGRAL_LIMIT_PRESET 50

#define PID_DONE_THRESHOLD 2

//-Others-------------------------------------------------------------------------------------------------------------------------//
#define LCD_PORT uart2
#define LCD_BACKLIGHT true
#define USING_LCD false

#define DRIVE_THRESHOLD 15
#define DRIVE_PID_CORRECTION_CYCLES 10
#define SLEW_GAIN 9.7692307692307692307692307692308
// = 254/(floor(maxTime / DRIVERCONTROL_LOOP_DELAY))

#define DRIVE_WIDTH 13.0
#define WHEEL_DIAMETER 3.25
#define PI 3.141592653589793
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
#define AUTON_LOOPS_DELAYS 1

//For easy array reference
#define LEFT 0
#define RIGHT 1

#endif
