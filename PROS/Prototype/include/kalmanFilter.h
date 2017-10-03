#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

float filterDrive[6];
float filterDriveL[6], filterDriveR[6];
float filterArmL[6], filterArmR[6];
//Remember to initialize arrays in init.c

int getSensor(float values[], int sensorValue);
/*
values[] array format:
KG(0), estimate(1), previousEstimate(2), errorEstimate(3), previousErrorEstimate(4), errorMeasurement(5)
*/

#endif
