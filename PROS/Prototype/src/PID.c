#include "main.h"

signed char PID(float values[], int target, bool useKI, bool useKD) {
	/*
	values[] array format:
	KP(0),  KI(1), KD(2), sensorValue(3), error(4), integral(5), integralLimit(6), lastError(7)
	*/
	values[4] = target - values[3];
	values[5] += (abs(values[5] + values[4]) < values[6]) ? values[4] : 0;
	values[5] = values[4] == 0 ? 0 : values[5];
	signed short shortOutput = (useKI && useKD) ? values[0] * values[4] + values[1] * values[5] + values[2] * (values[7] - values[4]) :
		useKI ? values[0] * values[4] + values[1] * values[5] :
		useKD ? values[0] * values[4] + values[2] * (values[7] - values[4]) :
		values[0] * values[4];
	values[7] = values[4];

	signed char output = abs(shortOutput) > 127 ? shortOutput < -127 ? -127 : 127 : shortOutput;

	return output;
}
