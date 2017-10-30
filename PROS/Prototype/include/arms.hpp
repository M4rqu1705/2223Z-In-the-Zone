#ifndef ARMS_HPP_
#define ARMS_HPP_

#include <API.hpp>
#include <utils.hpp>
#include <PID.hpp>

namespace arms {

	toggleButton buttonPressed, loaderButtonPressed;

	bool notDone;                                          //Declare booleans to indicate if the arm is done moving during autonomous
	enum position : uint_fast8_t { rightDown = 0, rightUp, rightLoader, leftLoader };    //Declare enumerated type variable to indicate arm positions
	position currentPosition;

	void changePositionTo(position state);
	void operatorControl();
}
#endif
