#ifndef CLAWS_H_
#define CLAWS_H_

bool clawButtonPressed, rightClawClosed, clawsDone;
unsigned char clawsCounter;
signed char clawLoutput, clawRoutput;

void clawsControl(bool state);

void clawsOperatorControl();

#endif
