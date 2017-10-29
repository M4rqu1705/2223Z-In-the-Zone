#ifndef PID_HPP_
#define PID_HPP_

#include <API.hpp>
#include <utils.hpp>

namespace pid {
	float PIDdrive[7], PIDarmL[7], PIDarmR[7];    //Declare PID arrays for drive and arms
	int_fast16_t output;

	int_fast8_t PID(float *values, int_fast16_t target, int_fast32_t sensorInput);

}
#endif
