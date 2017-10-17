#include "main.h"

uint_fast16_t getSensor(float values[], uint_fast16_t sensorValue) {
	/*
	values[] array format:
	KG(0), estimate(1), previousEstimate(2), errorEstimate(3), previousErrorEstimate(4), errorMeasurement(5)
	*/

	//Calculate Kalman Gain
	if (values[3] - values[5] != 0) values[0] = values[3] / (values[3] - values[5]);
	else values[0] = 0;

	//Calculate Estimate
	values[1] = sensorValue + values[0] * (values[2]-sensorValue);

	//Calculate Error in Estimate
	values[3] = (1 - values[0]) * values[4];

	values[2] = values[1]; values[4] = values[3];

	estimate = ceil(values[1]); //Convert float to int

	return estimate;
}
