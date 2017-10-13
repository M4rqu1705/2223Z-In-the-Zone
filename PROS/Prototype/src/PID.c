#include "main.h"

int_fast8_t PID(float values[], uint_fast16_t target) {
	/*
	values[] array format:
	KP(0),  KI(1), KD(2), sensorValue(3), error(4), integral(5), integralLimit(6), lastError(7)
	*/
	values[4] = target - values[3];
	values[5] += (abs(values[5] + values[4]) < values[6]) ? values[4] : 0;
	values[5] = values[4] == 0 ? 0 : values[5];

	output = values[0] * values[4] + values[1] * values[5] + values[2] * (values[7] - values[4]);

	values[7] = values[4];

	return WITHIN_RANGE(output);
}
