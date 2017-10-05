#include "main.h"

uint_fast16_t getSensor(float values[], uint_fast16_t sensorValue) {
	/*
	values[] array format:
	KG(0), estimate(1), previousEstimate(2), errorEstimate(3), previousErrorEstimate(4), errorMeasurement(5)
	*/

	values[0] = values[3] / (values[3] - values[5]);
	values[1] = values[2] + values[0] * (sensorValue - values[2]);
	values[3] = (1 - values[0]) * values[4];
	values[2] = values[1]; values[4] = values[3];

	uint_fast16_t estimate = ceil(values[1]); //Convert float to int

	return estimate;
}
