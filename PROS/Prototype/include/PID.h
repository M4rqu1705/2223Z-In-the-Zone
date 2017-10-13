#ifndef PID_H_
#define PID_H_

#include <API.h>

float PIDdrive[8];
float PIDarmL[8], PIDarmR[8];
//Remember to initialize arrays in init.c

int_fast16_t output;

int_fast8_t PID(float values[], uint_fast16_t target);
/*
values[] array format:
KP(0),  KI(1), KD(2), sensorValue(3), error(4), integral(5), integralLimit(6), lastError(7)
*/

#endif
