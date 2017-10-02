#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

float filterArmL[6], filterArmR[6];
//Remember to initialize arrays in init.c

int getSensor(float values[], int sensorValue);

#endif
