#ifndef CLAWS_HPP_
#define CLAWS_HPP_

#include <API.hpp>
#include <utils.hpp>

namespace claws {
	counterType counter;
	toggleButton buttonPressed;

	bool openRight, notDone;    //Declare booleans to indicate claw positions and if the claws are done moving during autonomous
	extern int_fast16_t outputs[2];     //Declare shorts to store the speed in which to move each on of the claws

	void changePositionTo(bool state);
	void operatorControl();

}

#endif
