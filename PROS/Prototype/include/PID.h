#ifndef PID_H_
#define PID_H_

float PIDdrive[8];
float PIDarmL[8], PIDarmR[8];
//Remember to initialize arrays in init.c

signed char PID(float values[], int target, bool useKI = true, bool useKD = true);
/*
values[] array format:
KP(0),  KI(1), KD(2), sensorValue(3), error(4), integral(5), integralLimit(6), lastError(7)
*/

#endif
