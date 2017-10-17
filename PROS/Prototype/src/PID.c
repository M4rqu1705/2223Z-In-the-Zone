#include "main.h"

int_fast8_t PID(float values[], uint_fast16_t target, uint_fast16_t sensorValue) {
	/*
	values[] array format:
	KP(0),  KI(1), KD(2), , error(3), integral(4), integralLimit(5), lastError(6)
	*/

	values[3] = target - sensorValue;
	values[4] += (abs(values[4] + values[3]) < values[5]) ? values[3] : 0;
	if (values[3] == 0) values[4] = 0;

	output = values[0] * values[3] + values[1] * values[4] + values[2] * (values[6] - values[3]);

	values[6] = values[3];

	return WITHIN_RANGE(output);
}
