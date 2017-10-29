/*#include <main.hpp>

#if USING_LCD

namespace LCD {
	void lcdSelect() {
		if (lcdReadButtons(LCD_PORT) == 4) {
			if (!lcdButtonsPressed[0]) {
				lcdButtonsPressed[0] = true;
				switch (currentMenu) {
				case mainMenu:
					currentMenu = batteryVoltageMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "Battery Voltage");
					output = "<     " + (std::string)(powerLevelMain() / 1000.0) + "V    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case batteryVoltageMenu:
					currentMenu = backupBatteryVoltageMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "Backup Battery V");
					output = "<     " + (std::string)(powerLevelBackup() / 1000.0) + "V    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case backupBatteryVoltageMenu:
					currentMenu = powerExpanderMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "Power Expander V");
					output = "<     " + (std::string)(analogRead(POWER_EXPANDER_STATUS) / POWER_EXPANDER_DIVISOR / 1000.0) + "V    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case powerExpanderMenu:
					currentMenu = autonomousMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "   Autonomous   ");
					if (currentCode == mogoAndCones)	output = "< MoGo + Cones >";
					else if (currentCode == cones)	output = "<     Cones    >";
					else if (currentCode == mogo)	output = "<  Mobile Goal >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case autonomousMenu:
					currentMenu = allianceColorMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "     Color      ");
					if (currentColor == red)	output = "<     Red      >";
					else if (currentColor == blue)	output = "<     Blue     >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case allianceColorMenu:
					currentMenu = startingSideMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "      Side      ");
					if (currentSide == left)	output = "<     Left     >";
					else if (currentSide == right)	output = "<     Right    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case startingSideMenu:
					currentMenu = mainMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "     2223-Z     ");
					lcdSetText(LCD_PORT, 1, "<      OK      >");
					break;
				}
			}
		}
		else lcdButtonsPressed[0] = false;

		if (lcdReadButtons(LCD_PORT) == 1) {
			if (!lcdButtonsPressed[2]) {
				lcdButtonsPressed[2] = true;
				switch (currentMenu) {
				case mainMenu:
					currentMenu = startingSideMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "      Side      ");
					if (currentSide == left)	output = "<     Left     >";
					else if (currentSide == right)	output = "<     Right    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case startingSideMenu:
					currentMenu = allianceColorMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "     Color      ");
					if (currentColor == red)	output = "<     Red      >";
					else if (currentColor == blue)	output = "<     Blue     >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case allianceColorMenu:
					currentMenu = autonomousMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "   Autonomous   ");
					if (currentCode == mogoAndCones)	output = "< MoGo + Cones >";
					else if (currentCode == cones)	output = "<     Cones    >";
					else if (currentCode == mogo)	output = "<  Mobile Goal >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case autonomousMenu:
					currentMenu = powerExpanderMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "Power Expander V");
					output = "<     " + (std::string)(analogRead(POWER_EXPANDER_STATUS) / POWER_EXPANDER_DIVISOR / 1000.0) + "V    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case powerExpanderMenu:
					currentMenu = backupBatteryVoltageMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "Backup Battery V");
					output = "<     " + (std::string)(powerLevelBackup() / 1000.0) + "V    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case backupBatteryVoltageMenu:
					currentMenu = batteryVoltageMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "Battery Voltage");
					output = "<     " + (std::string)(powerLevelMain() / 1000.0) + "V    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case batteryVoltageMenu:
					currentMenu = mainMenu;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "     2223-Z     ");
					lcdSetText(LCD_PORT, 1, "<      OK      >");
					break;
				}
			}
		}
		else lcdButtonsPressed[2] = false;

		if (lcdReadButtons(LCD_PORT) == 2) {
			if (!lcdButtonsPressed[1]) {
				lcdButtonsPressed[1] = true;
				switch (currentMenu) {
				case mainMenu:
					notDone = false;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "     2223-Z     ");
					break;

				case batteryVoltageMenu:
					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "Battery Voltage");
					output = "<     " + (std::string)(powerLevelMain() / 1000.0) + "V    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case backupBatteryVoltageMenu:
					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "Backup Battery V");
					output = "<     " + (std::string)(powerLevelBackup() / 1000.0) + "V    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case powerExpanderMenu:
					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "Power Expander V");
					output = "<     " + (std::string)(analogRead(POWER_EXPANDER_STATUS) / POWER_EXPANDER_DIVISOR / 1000.0) + "V    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case autonomousMenu:
					switch (currentCode) {
					case cones:
						currentCode = mogoAndCones;
						break;
					case mogoAndCones:
						currentCode = mogo;
						break;
					case mogo:
						currentCode = cones;
						break;
					}

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "   Autonomous   ");
					if (currentCode == mogoAndCones)	output = "< MoGo + Cones >";
					else if (currentCode == cones)	output = "<     Cones    >";
					else if (currentCode == mogo)	output = "<  Mobile Goal >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case allianceColorMenu:
					if (currentColor == blue) currentColor = red;
					else currentColor = blue;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "     Color      ");
					if (currentColor == red)	output = "<     Red      >";
					else if (currentColor == blue)	output = "<     Blue     >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				case startingSideMenu:
					if (currentSide == left) currentSide = right;
					else currentSide = left;

					lcdClear(LCD_PORT);

					lcdSetText(LCD_PORT, 0, "      Side      ");
					if (currentSide == left)	output = "<     Left     >";
					else if (currentSide == right)	output = "<     Right    >";
					lcdSetText(LCD_PORT, 1, output);
					break;

				}
			}
		}
		else lcdButtonsPressed[1] = false;
	}

}
#endif
*/
