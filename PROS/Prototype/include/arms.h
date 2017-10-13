#ifndef ARMS_H_
#define ARMS_H_

#include <API>

bool armsButtonPressed, armsLoaderButtonPressed, armsDone;

enum class armsPositions : uint_fast8_t { d = 0, u = 1, lr = 2, ll = 3 } currentArmPosition;

void armsOperatorControl();

void armsControl(armsPositions state);

#endif
