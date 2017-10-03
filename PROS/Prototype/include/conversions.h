#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

signed short map(signed short inNumber, signed short inMin, signed short inMax, signed short outMin, signed short outMax);

signed char withinRange(signed short inNumber);

signed short inchesOfTranslationToEncoderPulses(signed short inches);

signed short degreesOfRotationToGyroTicks(signed short degrees);

signed short degreesOfRotationToEncoderPulses(signed short degrees);

void rectifyOutputs(signed short *values, signed short speed, int leftSideSensor, int rightSideSensor);

#endif
