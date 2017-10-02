#ifndef PID_H_
#define PID_H_

float PIDarmL[8], PIDarmR[8];
//Remember to initialize arrays in init.c

signed char PID(float values[], int target, bool useKI = true, bool useKD = true);

#endif
