#ifndef ARMS_H_
#define ARMS_H_

#include <API.h>

bool armsButtonPressed, armsLoaderButtonPressed, armsDone;

enum class armsPositions { d = 0, u = 1, lr = 2, ll = 3 } currentArmPosition;

void armsOperatorControl();
void armsControl(armsPositions state);

#endif
