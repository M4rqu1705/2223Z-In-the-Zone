#ifndef DRIVE_H_
#define DRIVE_H_

#include <API.h>

int_fast8_t drivePowerOutput, driveTurnOutput, PIDoutput;
int_fast16_t driveOutputs[2], joystickDriveInputs[2];
bool driveInvertButtonPressed, driveDirectionNormal, driveDone;

enum class direction : uint_fast8_t { f = 0, b = 1, l = 2, r = 3 };

Encoder encoderLeft; Encoder encoderRight;
Gyro driveGyro;

void driveOperatorControl();

void drive(direction side, uint_fast16_t pulses, int_fast8_t speed, bool useGyro = true);

#endif
