#ifndef COMMONS_H_
#define COMMONS_H_

#pragma systemfile

//Motors and sensors setup
#define MOTOR_driveLF port4
#define MOTOR_driveLB port9
#define MOTOR_driveRB port2
#define MOTOR_driveRF port7
#define MOTOR_liftL port5
#define MOTOR_liftR port6
#define MOTOR_miniFourbarL port1
#define MOTOR_miniFourbarR port10
#define MOTOR_mobileGoal port3
#define MOTOR_goliathIntake port8

#define SENSOR_encoderL dgtl1
#define SENSOR_encoderR dgtl3
#define META_drivePulsesPerRevolution 360
#define SENSOR_gyro in1
#define SENSOR_lineTrackerLeft in4
#define SENSOR_lineTrackerRight in3
#define SENSOR_potentiometerMogo in5
#define SENSOR_potentiometerMiniFourbar in6
#define SENSOR_potentiometerLift in7

#define META_gyroCalibrationConstant 140

//Joystick setup
#define JOYSTICK_driveF Ch3
#define JOYSTICK_driveS Ch4
#define JOYSTICK_driveThreshold 15
#define JOYSTICK_lift Ch2
#define JOYSTICK_liftThreshold 15
#define JOYSTICK_mobileGoalE Btn6D
#define JOYSTICK_mobileGoalR Btn6U
#define JOYSTICK_miniFourbarU Btn5U
#define JOYSTICK_miniFourbarD Btn5D
#define JOYSTICK_goliathIntakeI Btn7D
#define JOYSTICK_goliathIntakeD Btn7L

//PID variables setup
//Format:[Unloaded = 0, Loaded = 1][positionPID = 0, speedPID = 1]
float PID_KPdrive[2][2] = {{0.45,0.5}, {0.4,0.5}};
float PID_KIdrive[2][2] = {{0,0.5}, {0,0.5}};
float PID_KDdrive[2][2] = {{0.04,0.0005}, {0.065,0.0005}};
float PID_KPdriveGyro[2] = {0.3, 0.3};
float PID_KIdriveGyro[2] = {0.3 , 0.3};
float PID_KDdriveGyro[2] = {0.04, 0.04};
#define PID_integralMaxDrive 127
#define PID_correctionCyclesDrive 20
#define PID_correctionThresholdDrive 5

#define META_driveWheelDiameter 4
#define META_driveWidth 13
#define META_slewRateMaxSlope 25.4
#define META_loopsDelay 20




#endif
