#ifndef	LCD_H_
#define LCD_H_

#include <API.h>

enum class lcdMenus : uint_fast8_t { main = 0, batteryVoltage = 1, backupBatteryVoltage = 2, autonomous = 3, allianceColor = 4, startingSide = 5 } currentMenu;

bool lcdButtonsPressed[3] = { false, false, false }, lcdReady;

enum class autonomousCodes : uint_fast8_t { mogoAndCones = 0, mogo = 1, cones = 2 };
enum class allianceColor : uint_fast8_t { red = 0, blue = 1 };
enum class startingSide : uint_fast8_t { left = 0, right = 1 };


autonomousCodes currentCode;
allianceColor currentColor;
startingSide currentSide;

char* lcdOutput[17];

void lcdSelect();

#endif
