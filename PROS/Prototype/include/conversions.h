#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <API.h>

#define ROUND(inNumber) (ceil(((inNumber)*(pow(10, 0))) - (0.49)) / (pow(10, 0)))
//http://www.cplusplus.com/forum/beginner/3600/

#define MAP(inNumber, inMin, inMax, outMin, outMax) ((inNumber)-(inMin)) * ((outMax)-(outMin)) / ((inMax)-(inMin)) + (outMin)

#define WITHIN_RANGE(inNumber) (abs(inNumber) > 127 ? inNumber < -127 ? -127 : 127 : inNumber)

#define INCHES_TRANSLATION_TO_ENCODER_PULSES(inches) (((WHEEL_DIAMETER)* PI)*((inches) / 360))

#define DEGREES_ROTATION_TO_GYRO_TICKS(degrees) (ROUND((degrees) * ((GYRO_FULL_ROTATION_TICKS) / 360)))

#define DEGREES_ROTATION_TO_ENCODER_PULSES(degrees) (ROUND((((degrees) / 360) * (DRIVE_WIDTH) * PI) / ((WHEEL_DIAMETER) / 360)))

//int_fast16_t MAP(int_fast16_t inNumber, int_fast16_t inMin, int_fast16_t inMax, int_fast16_t outMin, int_fast16_t outMax);
//int_fast8_t WITHIN_RANGE(int_fast16_t inNumber);
//int_fast16_t INCHES_TRANSLATION_TO_ENCODER_PULSES(int_fast16_t inches);
//int_fast16_t DEGREES_ROTATION_TO_GYRO_TICKS(int_fast16_t degrees);
//int_fast16_t DEGREES_ROTATION_TO_ENCODER_PULSES(int_fast16_t degrees);

void rectifyOutputs(int_fast32_t *values, int_fast32_t speed, uint_fast32_t leftSideSensor, uint_fast32_t rightSideSensor);

#endif
