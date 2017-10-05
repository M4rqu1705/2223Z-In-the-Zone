#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <API.h>

int_fast16_t map(int_fast16_t inNumber, int_fast16_t inMin, int_fast16_t inMax, int_fast16_t outMin, int_fast16_t outMax);

int_fast8_t withinRange(int_fast16_t inNumber);

int_fast16_t inchesOfTranslationToEncoderPulses(int_fast16_t inches);

int_fast16_t degreesOfRotationToGyroTicks(int_fast16_t degrees);

int_fast16_t degreesOfRotationToEncoderPulses(int_fast16_t degrees);

void rectifyOutputs(int_fast32_t *values, int_fast32_t speed, uint_fast32_t leftSideSensor, uint_fast32_t rightSideSensor);

#endif
