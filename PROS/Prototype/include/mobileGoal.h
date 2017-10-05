#ifndef MOBILEGOAL_H_
#define MOBILEGOAL_H_

#include <API.h>

bool mogoButtonPressed, mogoRetracted, mogoDone;
uint_fast8_t mogoCounter;
int_fast8_t mogoLoutput, mogoRoutput;

void mobileGoalControl(bool state);

void mobileGoalOperatorControl();

#endif
