#ifndef CLAWS_H_
#define CLAWS_H_

#include <API.h>

bool clawButtonPressed, rightClawClosed, clawsDone;
uint_fast8_t clawsCounter;
int_fast8_t clawLoutput, clawRoutput;

void clawsControl(bool state);

void clawsOperatorControl();

#endif
