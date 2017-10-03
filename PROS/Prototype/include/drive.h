#ifndef DRIVE_H_
#define DRIVE_H_

#include "API.h"

signed char drivePowerOutput, driveTurnOutput, PIDoutput;
signed short driveOutputs[2], joystickDriveInputs[2];
bool driveInvertButtonPressed, driveDirectionNormal, driveDone;

enum direction { f, b, l, r };

Encoder encoderLeft; Encoder encoderRight;
Gyro driveGyro;

void driveOperatorControl();

void drive(direction side, unsigned short pulses, signed char speed, bool useGyro = true);

#endif
