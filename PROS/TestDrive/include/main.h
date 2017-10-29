#ifndef MAIN_H_
#define MAIN_H_

#include <API.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_DRIVE_LB 1
#define MOTOR_DRIVE_LF 2
#define MOTOR_DRIVE_RF 9
#define MOTOR_DRIVE_RB 10

#define JOYSTICK_DRIVE_F 2
#define JOYSTICK_DRIVE_S 1

void autonomous();

void initializeIO();

void initialize();

short driveOutputs[2];
void operatorControl();

// End C++ export structure
#ifdef __cplusplus
}
#endif

#endif
