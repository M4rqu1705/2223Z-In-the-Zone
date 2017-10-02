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
#define JOYSTICK_CLAW 5
#define JOYSTICK_CLAW_BUTTON JOY_DOWN
#define JOYSTICK_MOGO 6
#define JOYSTICK_MOGO_BUTTON JOY_UP



#define DRIVE_THRESHOLD 15
#define SLEW_GAIN 9

#define ARM_UP 1024
#define ARM_DOWN 0

#define CLAW_CYCLES 5

#define MOGO_CYCLES 10

#define LOOP_DELAY 15


#endif