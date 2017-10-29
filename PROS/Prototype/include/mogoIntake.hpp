#ifndef MOGO_INTAKE_HPP_
#define MOGO_INTAKE_HPP_

#include <API.hpp>
#include <utils.hpp>

namespace mobileGoalIntake {
	counterType counter;
	toggleButton buttonPressed;

	bool extend, notDone;      //Declare booleans to indicate if mobile goal is currently retracted or not (if, when activated, it will respectively extend or retract) and to indicate if the mobile goal intake is done moving during autonomous
	int_fast8_t outputs[2];    //Declare shorts to store the speed in which to move each side of the Mobile Goal intake

	void changePositionTo(bool state);
	void operatorControl();

}

#endif
