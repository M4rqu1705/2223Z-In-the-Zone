#ifndef COMMONS_H_
#define COMMONS_H_

#pragma systemfile

//Motors and sensors setup

#define MOTOR_miniFourbarR port1
#define MOTOR_driveRB port2
#define MOTOR_liftR port3
#define MOTOR_driveRF port4
#define MOTOR_mobileGoal port5
#define MOTOR_goliathIntake port6
#define MOTOR_driveLF port7
#define MOTOR_liftL port8
#define MOTOR_driveLB port9
#define MOTOR_miniFourbarL port10

// Motor ports: 					 port1, port2, port3, port4, port5, port6, port7, port8, port9, port10
bool MOTOR_inverted[10] = {true , false, false, false, false, true , true,  true , false, false};

#define SENSOR_encoderL dgtl1
#define SENSOR_encoderL2 dgtl2
#define SENSOR_encoderR dgtl3
#define SENSOR_encoderR2 dgtl4
#define META_encoderLInverted true
#define META_encoderRInverted true
#define META_drivePulsesPerRevolution 540
#define SENSOR_gyro in5
#define SENSOR_lineTrackerLeft in6
#define SENSOR_lineTrackerRight in7
#define SENSOR_potentiometerMogo in4
#define SENSOR_potentiometerMiniFourbar in3
#define SENSOR_potentiometerLiftR in1
#define SENSOR_potentiometerLiftL in2

#define META_gyroCalibrationConstant 136

//Joystick setup
#define JOYSTICK_driveF Ch3
#define JOYSTICK_driveS Ch4
#define JOYSTICK_driveThreshold 30
#define JOYSTICK_lift Ch2
#define JOYSTICK_liftThreshold 15
#define JOYSTICK_mobileGoalE Btn6D
#define JOYSTICK_mobileGoalR Btn6U
#define JOYSTICK_coneU Btn5U
#define JOYSTICK_coneD Btn5D
#define JOYSTICK_goliathIntakeI Btn7D
#define JOYSTICK_goliathIntakeD Btn8D
#define JOYSTICK_miniFourbarU Btn7R
#define JOYSTICK_miniFourbarD Btn8R


//PID variables setup
//Format:[Unloaded = 0, Loaded = 1][positionPID = 0, speedPID = 1]
float PID_KPdrive[2][2] = {{0.2, 0}, {0, 0}};
float PID_KIdrive[2][2] = {{0,0}, {0,0}};
float PID_KDdrive[2][2] = {{0.02,0}, {0,0}};
float PID_KPdriveGyro[2] = {0.35, 0};
float PID_KIdriveGyro[2] = {0 , 0};
float PID_KDdriveGyro[2] = {0.035, 0};
float PID_KPlift[2] = {0.25, 0.2}, PID_KIlift[2] = {0.1, 0.1}, PID_KDlift[2] = {0, 0};
float PID_KPmobileGoalIntake[2] = {0.1, 0.1}, PID_KImobileGoalIntake[2] = {0, 0}, PID_KDmobileGoalIntake[2] = {0, 0.05};
float PID_KPminiFourbar[2] = {0.075, 0.1}, PID_KIminiFourbar[2] = {0, 0}, PID_KDminiFourbar[2] = {0, 0.5};

#define PID_correctionCyclesDrive 20
#define PID_correctionCyclesLift 20
#define PID_correctionCyclesMobileGoal 20
#define PID_correctionCyclesMiniFourbar 30
#define PID_correctionCyclesGoliathIntake 20
#define PID_correctionThresholdDrive 5

#define META_driveWheelDiameter 4.125
#define META_driveWidth 13
float META_driveRectificationConstant = 3;
#define META_slewRateMaxSlope 25.4
#define META_loopsDelay 20

#define META_miniFourbarUp 1750
#define META_miniFourbarDown 4030

#define META_mobileGoalIntakeRetracted 550
#define META_mobileGoalIntakeExtended 2500


#endif
