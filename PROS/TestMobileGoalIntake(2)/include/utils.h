#ifndef UTILS_H_INCLUDED_
#define UTILS_H_INCLUDED_

#include "API.h"
#include "commons.h"

signed int MAP(signed int inNumber, signed int inMax, signed int inMin, signed int outMax, signed int outMin);
//Retreived from https://www.arduino.cc/en/Reference/Map

signed int ROUND(float inNumber);

signed char CLAMP(signed int inNumber);

typedef void (*returnFunction)(void);
void toggleButton(unsigned char buttonGroup, unsigned char button, bool &buttonToggleVariable, returnFunction function);

namespace PID{
  signed char calculatePID(float *values, signed int target, signed int current);
}
#endif
