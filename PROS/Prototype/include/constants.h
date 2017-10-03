#ifndef CONSTANTS_H_
#define CONSTANTS_H_


//-Motor ports-------------------------------------------------------------------------------------------------------------------//
#define MOTOR_MOGO_L 1
#define MOTOR_DRIVE_LF 2
#define MOTOR_DRIVE_LB 3
#define MOTOR_ARM_L 4
#define MOTOR_CLAW_L 5
#define MOTOR_CLAW_R 6
#define MOTOR_ARM_R 7
#define MOTOR_DRIVE_RB 8
#define MOTOR_DRIVE_RF 9
#define MOTOR_MOGO_R 10

//-Sensor ports------------------------------------------------------------------------------------------------------------------//
#define SENSOR_ENCODER_L 1
#define SENSOR_ENCODER_L_INVERTED false
#define SENSOR_ENCODER_R 3
#define SENSOR_ENCODER_R_INVERTED true
#define SENSOR_GYRO 8

#define SENSOR_POT_L 2
#define SENSOR_POT_R 3

//-Joystick ports----------------------------------------------------------------------------------------------------------------//
#define JOYSTICK_DRIVE_F 3
#define JOYSTICK_DRIVE_S 1
#define JOYSTICK_DRIVE_INVERT 8
#define JOYSTICK_DRIVE_INVERT_BUTTON JOY_LEFT

#define JOYSTICK_ARM 5
#define JOYSTICK_ARM_BUTTON JOY_UP
#define JOYSTICK_ARM_LOADER 5
#define JOYSTICK_ARM_LOADER_BUTTON JOY_DOWN

#define JOYSTICK_CLAWS 6
#define JOYSTICK_CLAWS_BUTTON JOY_DOWN
#define JOYSTICK_MOGO 6
#define JOYSTICK_MOGO_BUTTON JOY_UP

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
#define DRIVE_FILTERS_KG_PRESET 0
#define DRIVE_FILTERS_ESTIMATE_PRESET 0
#define DRIVE_FILTERS_PREVIOUS_ESTIMATE_PRESET 0
#define DRIVE_FILTERS_ERROR_ESTIMATE_PRESET 5
#define DRIVE_FILTERS_PREVIOUS_ERROR_ESTIMATE_PRESET 5
#define DRIVE_FILTERS_ERROR_MEASUREMENT_PRESET 5

#define ARM_FILTERS_KG_PRESET 0
#define ARM_FILTERS_ESTIMATE_PRESET 0
#define ARM_FILTERS_PREVIOUS_ESTIMATE_PRESET 0
#define ARM_FILTERS_ERROR_ESTIMATE_PRESET 5
#define ARM_FILTERS_PREVIOUS_ERROR_ESTIMATE_PRESET 5
#define ARM_FILTERS_ERROR_MEASUREMENT_PRESET 5

//-Others-------------------------------------------------------------------------------------------------------------------------//
#define LCD_PORT uart1
#define LCD_BACKLIGHT true

#define DRIVE_THRESHOLD 15
#define SLEW_GAIN 9
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


#endif
