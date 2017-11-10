#ifndef PID_H_INCLUDED_
#define PID_H_INCLUDED_

#include "main.h"

class PID {
	public:
	PID(float _KP, float _KI, float _KD, float _integralLimit);
	void resetVariables();
	int_fast8_t calculatePID(int_fast16_t target, int_fast32_t sensorInput);

	private:
		const float KP, KI, KD, integralLimit;
		int_fast16_t error, lastError, integral;
		int_fast8_t output;

};

/*
namespace pid {
	float PIDdrive[7], PIDarmL[7], PIDarmR[7];    //Declare PID arrays for drive and arms
	int_fast16_t output;

	int_fast8_t PID(float *values, int_fast16_t target, int_fast32_t sensorInput);

}*/
#endif
