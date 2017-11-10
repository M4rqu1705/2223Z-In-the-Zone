#ifndef UTILS_H_INCLUDED_
#define UTILS_H_INCLUDED_

#include "API.h"
#include "commons.h"

int_fast32_t ROUND(float inNumber);
int_fast16_t MAP(int_fast16_t inNumber, int_fast16_t inMin, int_fast16_t inMax, int_fast16_t outMin, int_fast16_t outMax);
int_fast8_t CLAMP(int_fast16_t inNumber);
int_fast8_t SIGN(int_fast8_t inNumber, float multiplier = 1);

uint_fast16_t INCHES_TRANSLATION_TO_ENCODER_PULSES(float inches);
int_fast16_t DEGREES_ROTATION_TO_GYRO_TICKS(float targetDegrees);
int_fast16_t DEGREES_ROTATION_TO_ENCODER_PULSES(float targetDegrees);

void rectifyOutputsEncoder(int_fast16_t *values, int_fast8_t speed, int_fast16_t leftSideSensor, int_fast16_t rightSideSensor);
void rectifyDriveGyro(int_fast16_t *values, int_fast8_t speed, int_fast16_t gyroSensor);

#endif
