#include <PID.hpp>

namespace pid{

int_fast8_t PID(float *values, int_fast16_t target, int_fast32_t sensorInput) {
	//values[] array format: KP(0),  KI(1), KD(2), error(3), integral(4), integralLimit(5), lastError(6)

	//Calculate error
	values[3] = (float)(target - sensorInput);

	//Calculate integral if within the integralLimit range
	values[4] += (abs((int)(values[4])) + abs((int)(values[3])) < values[5]) ? values[3] : 0;
	//In the case that error is 0, reset the integral
	if (values[3] == 0) values[4] = 0;

	//Calculate the output adding KP by error, KI by integral and KD by derivative
	//Just so the Integral doesn't grow too fast, map an exagerated number to fit values from -127 to 127
	//Calculate the derivative here so there is no need to store it in a variable
	output = values[0] * values[3] + MAP((values[1] * values[4]), -(values[5]), values[5], -127, 127) + values[2] * (values[3] - values[6]);

	values[6] = values[3];

	return CLAMP(output);
}

}
