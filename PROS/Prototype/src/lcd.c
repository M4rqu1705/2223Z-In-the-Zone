#include "main.h"

void lcdDisplay() {
	switch (currentDisplay) {
	case batteryVoltage:
		lcdClear(LCD_PORT);

		*lcdOutput = "<     %1.3f     >", ((float)powerLevelMain() / 1000);
		lcdSetText(LCD_PORT, 1, "Battery Voltage");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
		break;

	case backupBatteryVoltage:
		lcdClear(LCD_PORT);

		*lcdOutput = "<     %1.3f     >", ((float)powerLevelBackup() / 1000);
		lcdSetText(LCD_PORT, 1, "Backup Battery V");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
			break;

	case auton:
		if (currentCode == mogoAndCones)	*lcdOutput = "< MoGo + Cones >";
		else if (currentCode == cones)	*lcdOutput = "<     Cones    >";
		else if (currentCode == mogo)	*lcdOutput = "<  Mobile Goal >";

		lcdClear(LCD_PORT);

		lcdSetText(LCD_PORT, 1, "  Current Auton ");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
		break;
	case color:
		if (currentColor == red)	*lcdOutput = "<     Red      >";
		else if (currentColor == blue)	*lcdOutput = "<     Blue     >";

		lcdClear(LCD_PORT);

		lcdSetText(LCD_PORT, 1, "  Current Auton ");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
			break;
	case side:
		if (currentSide == right)	*lcdOutput = "<     Right    >";
		else if (currentSide == left)	*lcdOutput = "<     Left     >";

		lcdClear(LCD_PORT);

		lcdSetText(LCD_PORT, 1, "  Current Side  ");
		lcdSetText(LCD_PORT, 2, *lcdOutput);
			break;

	default:
		lcdClear(LCD_PORT);
		lcdSetText(LCD_PORT, 1, "     2223-Z     ");
		lcdSetText(LCD_PORT, 2, "<      OK      >");
		break;
	}
}

void lcdSelect() {
	if (lcdReadButtons(LCD_PORT) == 1) {
		if (!lcdButtonsPressed[2]) {
			lcdButtonsPressed[2] = true;
			currentDisplay = (currentDisplay == main) ? batteryVoltage :
												(currentDisplay == batteryVoltage) ? backupBatteryVoltage :
												(currentDisplay == backupBatteryVoltage) ? auton :
												(currentDisplay == auton) ? color :
												(currentDisplay == color) ? side : main;
			lcdDisplay();
		}
	}
	else lcdButtonsPressed[2] = false;

	if (lcdReadButtons(LCD_PORT) == 4) {
		if (!lcdButtonsPressed[0]) {
			lcdButtonsPressed[2] = true;
			currentDisplay = (currentDisplay == main) ? side :
												(currentDisplay == side) ? color :
												(currentDisplay == color) ? auton :
												(currentDisplay == auton) ? backupBatteryVoltage :
												(currentDisplay == backupBatteryVoltage) ? batteryVoltage : main;
			lcdDisplay();
		}
	}
	else lcdButtonsPressed[0] = false;

	if (lcdReadButtons(LCD_PORT) == 2) {
		if (!lcdButtonsPressed[1]) {
			lcdButtonsPressed[1] = true;
			switch (currentDisplay) {
			case main:
				lcdReady = true;
				break;
			case batteryVoltage:
				lcdDisplay();
				break;
			case backupBatteryVoltage:
				lcdDisplay();
				break;
			case auton:
				currentCode = (currentCode == cones) ? mogoAndCones : (currentCode == mogoAndCones) ? mogo : cones ;
				lcdDisplay();
				break;
			case color:
				currentColor = currentColor == blue ? red : blue;
				lcdDisplay();
				break;
			case side:
				currentSide = currentSide == left ? right : left;
				lcdDisplay();
				break;
			}
		}
	}
	else lcdButtonsPressed[1] = false;

}
