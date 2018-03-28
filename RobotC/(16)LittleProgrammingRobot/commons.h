#ifndef COMMONS_H_
#define COMMONS_H_

#pragma systemfile

//Motors and sensors setup
#define MOTOR_driveLF port9
#define MOTOR_driveLB port10
#define MOTOR_driveRB port1
#define MOTOR_driveRF port2

#define SENSOR_encoderL dgtl1
#define SENSOR_encoderR dgtl3
#define SENSOR_gyro in1
#define SENSOR_accelerometer in2
#define SENSOR_lineTrackerLeft in4
#define SENSOR_lineTrackerRight in3

#define META_gyroCalibrationConstant 140
#define META_drivePulsesPerRevolution 720
#define META_encoderLInverted false
#define META_encoderRInverted true

//Joystick setup
#define JOYSTICK_driveF Ch3
#define JOYSTICK_driveS Ch1
#define JOYSTICK_driveThreshold 20

//PID variables setup
//Format:[Unloaded = 0, Loaded = 1][positionPID = 0, speedPID = 1]
float PID_KPdrive[2][2] = {{0.45,0.5}, {0.4,0.5}};
float PID_KIdrive[2][2] = {{0,0.5}, {0,0.5}};
float PID_KDdrive[2][2] = {{0.04,0.0005}, {0.065,0.0005}};
float PID_KPdriveGyro[2] = {0.3, 0.3};
float PID_KIdriveGyro[2] = {0.3 , 0.3};
float PID_KDdriveGyro[2] = {0.04, 0.04};
float PID_KVdrive = 0.52917;	//Calculated from 127/240
#define PID_integralMaxDrive 127
#define PID_correctionCyclesDrive 20
#define PID_correctionThresholdDrive 5

#define META_driveWheelDiameter 4
#define META_driveWidth 13
#define META_driveSlewRateMaxSlope 25.4

#define META_loopsDelay 500


#endif
