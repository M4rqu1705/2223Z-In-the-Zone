#ifndef	LCD_H_
#define LCD_H_

enum lcdDisplays { main=0, batteryVoltage=1, backupBatteryVoltage=2, auton=3, color=4, side=5 };

bool lcdButtonsPressed[3] = { false, false, false }, lcdReady;

enum autonCodes {mogoAndCones=0, mogo=1, cones=2};
enum allianceColor {red=0, blue=1};
enum robotSide {left=0, right=1};

lcdDisplays currentDisplay;
autonCodes currentCode;
allianceColor currentColor;
robotSide currentSide;

char *lcdOutput[17];

void lcdDisplay();
void lcdSelect();

#endif
