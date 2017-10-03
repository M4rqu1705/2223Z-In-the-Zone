#ifndef MOBILEGOAL_H_
#define MOBILEGOAL_H_

bool mogoButtonPressed, mogoRetracted, mogoDone;
unsigned char mogoCounter;
signed char mogoLoutput, mogoRoutput;

void mobileGoalControl(bool state);

void mobileGoalOperatorControl();

#endif
