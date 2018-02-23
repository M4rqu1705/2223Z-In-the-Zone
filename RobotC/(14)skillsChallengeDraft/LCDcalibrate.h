/*   LCDcalibrate.h - "Driver" for LCD PID constant calibration             *
*    Copyright (C) <2018>  Marcos Ricardo Pesante Colón                     *
*                                                                           *
*    This program is free software: you can redistribute it and/or modify   *
*    it under the terms of the GNU General Public License as published by   *
*    the Free Software Foundation, either version 3 of the License, or      *
*    (at your option) any later version.                                    *
*                                                                           *
*    This program is distributed in the hope that it will be useful,        *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of         *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
*    GNU General Public License for more details.                           *
*                                                                           *
*    You should have received a copy of the GNU General Public License      *
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#ifndef LCD_CALIBRATE_H_
#define LCD_CALIBRATE_H_

#pragma systemfile

#ifdef META_usingLCD    //Only include code if META_usingLCD is defined in the constants.h file
bool LCD_retracting = false, LCD_mogoLoaded = false, LCD_gyro = false;
enum LCD_menuMode{Loaded, Retracting, menuGyro, Constants, ManipulateConstants};
enum LCD_PIDconstantsEnum {driveSubsKP, driveSubsKI, driveSubsKD,
	mogoIntakeSubsKP, mogoIntakeSubsKI, mogoIntakeSubsKD,
	armSubsKP, armSubsKI, armSubsKD};

LCD_menuMode LCD_currentMenuMode = Loaded;
LCD_PIDconstantsEnum LCD_currentConstant = driveSubsKP;

bool LCD_buttonsPressed[3];
string LCD_output;    //Declare variable in which the formatted string containing a variable LCD output will be stored

void LCD_init();
void LCD_refresh();
void LCD_calibrate();

void LCD_addToConstant(float *constant){
	*constant+=0.05;
}

void LCD_substractToConstant(float *constant){
	*constant-=0.05;
}

void LCD_addToConstant(float *constant, float add){
	*constant += add;
}

void LCD_substractToConstant(float *constant, float substract){
	*constant -= substract;
}

void LCD_init(){
	clearLCDLine(0);                  //Clear LCD
	clearLCDLine(1);                  //Clear LCD
	bLCDBacklight = META_LCDbacklight;    //Turn backlight on or off based on META_LCDbacklight constant

	LCD_retracting = false;
	LCD_mogoLoaded = false;
	LCD_gyro = false;
	LCD_currentConstant = driveSubsKP;

	//None of the LCD buttons have been previously pressed
	LCD_buttonsPressed[0] = false;
	LCD_buttonsPressed[1] = false;
	LCD_buttonsPressed[2] = false;

}

void LCD_refresh(){
	if(LCD_currentMenuMode == Retracting){
		displayLCDCenteredString(0, "Retracting");
		if(LCD_retracting) LCD_output = "True";
		else LCD_output = "False";
		displayLCDCenteredString(1, LCD_output);
	}
	else if(LCD_currentMenuMode == Loaded){
		displayLCDCenteredString(0, "Loaded");
		if(LCD_mogoLoaded) LCD_output = "True";
		else LCD_output = "False";
		displayLCDCenteredString(1, LCD_output);
	}
	else if(LCD_currentMenuMode == menuGyro){
		displayLCDCenteredString(0, "menuGyro");
		if(LCD_gyro) LCD_output = "True";
		else LCD_output = "False";
		displayLCDCenteredString(1, LCD_output);
	}
	else if(LCD_currentMenuMode == Constants || LCD_currentMenuMode == ManipulateConstants){
		switch(LCD_currentConstant){
		case driveSubsKP:
			displayLCDCenteredString(0, "Drive KP");
			if(LCD_mogoLoaded && LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KPdriveGyro[1][0]);
			else if(LCD_mogoLoaded && !LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KPdrive[1][0]);
			else if(!LCD_mogoLoaded && LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KPdriveGyro[0][0]);
			else if(!LCD_mogoLoaded && !LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KPdrive[0][0]);
			displayLCDCenteredString(1, LCD_output);
			break;
		case driveSubsKI:
			displayLCDCenteredString(0, "Drive KI");
			if(LCD_mogoLoaded && LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KIdriveGyro[1][0]);
			else if(LCD_mogoLoaded && !LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KIdrive[1][0]);
			else if(!LCD_mogoLoaded && LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KIdriveGyro[0][0]);
			else if(!LCD_mogoLoaded && !LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KIdrive[0][0]);
			displayLCDCenteredString(1, LCD_output);
			break;
		case driveSubsKD:
			displayLCDCenteredString(0, "Drive KD");
			if(LCD_mogoLoaded && LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KDdriveGyro[1][0]);
			else if(LCD_mogoLoaded && !LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KDdrive[1][0]);
			else if(!LCD_mogoLoaded && LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KDdriveGyro[0][0]);
			else if(!LCD_mogoLoaded && !LCD_gyro) sprintf(LCD_output, "%1.3f", &PID_KDdrive[0][0]);
			displayLCDCenteredString(1, LCD_output);
			break;

		case mogoIntakeSubsKP:
			displayLCDCenteredString(0, "Mogo KP");
			if(LCD_mogoLoaded && LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KPmobileGoalIntake[1][1]);
			else if(LCD_mogoLoaded && !LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KPmobileGoalIntake[1][0]);
			else if(!LCD_mogoLoaded && LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KPmobileGoalIntake[0][1]);
			else if(!LCD_mogoLoaded && !LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KPmobileGoalIntake[0][0]);
			displayLCDCenteredString(1, LCD_output);
			break;
		case mogoIntakeSubsKI:
			displayLCDCenteredString(0, "Mogo KI");
			if(LCD_mogoLoaded && LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KImobileGoalIntake[1][1]);
			else if(LCD_mogoLoaded && !LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KImobileGoalIntake[1][0]);
			else if(!LCD_mogoLoaded && LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KImobileGoalIntake[0][1]);
			else if(!LCD_mogoLoaded && !LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KImobileGoalIntake[0][0]);
			displayLCDCenteredString(1, LCD_output);
			break;
		case mogoIntakeSubsKD:
			displayLCDCenteredString(0, "Mogo KD");
			if(LCD_mogoLoaded && LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KDmobileGoalIntake[1][1]);
			else if(LCD_mogoLoaded && !LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KDmobileGoalIntake[1][0]);
			else if(!LCD_mogoLoaded && LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KDmobileGoalIntake[0][1]);
			else if(!LCD_mogoLoaded && !LCD_retracting) sprintf(LCD_output, "%1.3f", &PID_KDmobileGoalIntake[0][0]);
			displayLCDCenteredString(1, LCD_output);
			break;

		case armSubsKP:
			displayLCDCenteredString(0, "Arm KP");
			if(LCD_mogoLoaded) sprintf(LCD_output, "%1.3f", &PID_KParm[1]);
			else if(!LCD_mogoLoaded) sprintf(LCD_output, "%1.3f", &PID_KParm[0]);
			displayLCDCenteredString(1, LCD_output);
			break;
		case armSubsKI:
			displayLCDCenteredString(0, "Arm KI");
			if(LCD_mogoLoaded) sprintf(LCD_output, "%1.3f", &PID_KIarm[1]);
			else if(!LCD_mogoLoaded) sprintf(LCD_output, "%1.3f", &PID_KIarm[0]);
			displayLCDCenteredString(1, LCD_output);
			break;
		case armSubsKD:
			displayLCDCenteredString(0, "Arm KD");
			if(LCD_mogoLoaded) sprintf(LCD_output, "%1.3f", &PID_KDarm[1]);
			else if(!LCD_mogoLoaded) sprintf(LCD_output, "%1.3f", &PID_KDarm[0]);
			displayLCDCenteredString(1, LCD_output);
			break;
		}
	}
}


void LCD_calibrate(){		//It is in reality used for calibrating PID
	switch(nLCDButtons){		//If Middle Button Pressed
	case (TControllerButtons)2:
		if(!LCD_buttonsPressed[1]){
			LCD_buttonsPressed[1] = true;
			if(LCD_currentMenuMode == Loaded){
				LCD_currentMenuMode = Retracting;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == Retracting){
				LCD_currentMenuMode = menuGyro;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == menuGyro){
				LCD_currentMenuMode = Constants;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == Constants){
				LCD_currentMenuMode = ManipulateConstants;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == ManipulateConstants){
				LCD_currentMenuMode = Loaded;
				LCD_refresh();
			}
		}
		break;

	case (TControllerButtons)1:	//If left button pressed
		if(!LCD_buttonsPressed[0]){
			LCD_buttonsPressed[0] = true;
			if(LCD_currentMenuMode == Loaded){
				LCD_mogoLoaded = !LCD_mogoLoaded;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == Retracting){
				LCD_retracting = !LCD_retracting;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == menuGyro){
				LCD_gyro = !LCD_gyro;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == Constants){
				switch(LCD_currentConstant){
				case driveSubsKP:
					LCD_currentConstant = armSubsKD;
					LCD_refresh();
					break;
				case driveSubsKI:
					LCD_currentConstant = driveSubsKP;
					LCD_refresh();
					break;
				case driveSubsKD:
					LCD_currentConstant = driveSubsKI;
					LCD_refresh();
					break;
				case mogoIntakeSubsKP:
					LCD_currentConstant = driveSubsKD;
					LCD_refresh();
					break;
				case mogoIntakeSubsKI:
					LCD_currentConstant = mogoIntakeSubsKP;
					LCD_refresh();
					break;
				case mogoIntakeSubsKD:
					LCD_currentConstant = mogoIntakeSubsKI;
					LCD_refresh();
					break;
				case armSubsKP:
					LCD_currentConstant = mogoIntakeSubsKD;
					LCD_refresh();
					break;
				case armSubsKI:
					LCD_currentConstant = armSubsKP;
					LCD_refresh();
					break;
				case armSubsKD:
					LCD_currentConstant = armSubsKI;
					LCD_refresh();
					break;
				}
			}
			else if(LCD_currentMenuMode == ManipulateConstants){
				switch(LCD_currentConstant){
				case driveSubsKP:
					if(LCD_mogoLoaded && LCD_gyro) LCD_substractToConstant(&PID_KPdriveGyro[1][0]);
					else if(LCD_mogoLoaded && !LCD_gyro) LCD_substractToConstant(&PID_KPdrive[1][0]);
					else if(!LCD_mogoLoaded && LCD_gyro) LCD_substractToConstant(&PID_KPdriveGyro[0][0]);
					else if(!LCD_mogoLoaded && !LCD_gyro) LCD_substractToConstant(&PID_KPdrive[0][0]);
					LCD_refresh();
					break;
				case driveSubsKI:
					if(LCD_mogoLoaded && LCD_gyro) LCD_substractToConstant(&PID_KIdriveGyro[1][0]);
					else if(LCD_mogoLoaded && !LCD_gyro) LCD_substractToConstant(&PID_KIdrive[1][0]);
					else if(!LCD_mogoLoaded && LCD_gyro) LCD_substractToConstant(&PID_KIdriveGyro[0][0]);
					else if(!LCD_mogoLoaded && !LCD_gyro) LCD_substractToConstant(&PID_KIdrive[0][0]);
					LCD_refresh();
					break;
				case driveSubsKD:
					if(LCD_mogoLoaded && LCD_gyro) LCD_substractToConstant(&PID_KDdriveGyro[1][0], 0.005);
					else if(LCD_mogoLoaded && !LCD_gyro) LCD_substractToConstant(&PID_KDdrive[1][0], 0.005);
					else if(!LCD_mogoLoaded && LCD_gyro) LCD_substractToConstant(&PID_KDdriveGyro[0][0], 0.005);
					else if(!LCD_mogoLoaded && !LCD_gyro) LCD_substractToConstant(&PID_KDdrive[0][0], 0.005);
					LCD_refresh();
					break;

				case mogoIntakeSubsKP:
					if(LCD_mogoLoaded && LCD_retracting) LCD_substractToConstant(&PID_KPmobileGoalIntake[1][1]);
					else if(LCD_mogoLoaded && !LCD_retracting) LCD_substractToConstant(&PID_KPmobileGoalIntake[1][0]);
					else if(!LCD_mogoLoaded && LCD_retracting) LCD_substractToConstant(&PID_KPmobileGoalIntake[0][1]);
					else if(!LCD_mogoLoaded && !LCD_retracting) LCD_substractToConstant(&PID_KPmobileGoalIntake[0][0]);
					LCD_refresh();
					break;
				case mogoIntakeSubsKI:
					if(LCD_mogoLoaded && LCD_retracting) LCD_substractToConstant(&PID_KImobileGoalIntake[1][1]);
					else if(LCD_mogoLoaded && !LCD_retracting) LCD_substractToConstant(&PID_KImobileGoalIntake[1][0]);
					else if(!LCD_mogoLoaded && LCD_retracting) LCD_substractToConstant(&PID_KImobileGoalIntake[0][1]);
					else if(!LCD_mogoLoaded && !LCD_retracting) LCD_substractToConstant(&PID_KImobileGoalIntake[0][0]);
					LCD_refresh();
					break;
				case mogoIntakeSubsKD:
					if(LCD_mogoLoaded && LCD_retracting) LCD_substractToConstant(&PID_KDmobileGoalIntake[1][1], 0.005);
					else if(LCD_mogoLoaded && !LCD_retracting) LCD_substractToConstant(&PID_KDmobileGoalIntake[1][0], 0.005);
					else if(!LCD_mogoLoaded && LCD_retracting) LCD_substractToConstant(&PID_KDmobileGoalIntake[0][1], 0.005);
					else if(!LCD_mogoLoaded && !LCD_retracting) LCD_substractToConstant(&PID_KDmobileGoalIntake[0][0], 0.005);
					LCD_refresh();
					break;

				case armSubsKP:
					if(LCD_mogoLoaded) LCD_substractToConstant(&PID_KParm[1]);
					else if(!LCD_mogoLoaded) LCD_substractToConstant(&PID_KParm[0]);
					LCD_refresh();
					break;
				case armSubsKI:
					if(LCD_mogoLoaded) LCD_substractToConstant(&PID_KIarm[1]);
					else if(!LCD_mogoLoaded) LCD_substractToConstant(&PID_KIarm[0]);
					LCD_refresh();
					break;
				case armSubsKD:
					if(LCD_mogoLoaded) LCD_substractToConstant(&PID_KDarm[1], 0.005);
					else if(!LCD_mogoLoaded) LCD_substractToConstant(&PID_KDarm[0], 0.005);
					LCD_refresh();
					break;
				}
			}
		}
		break;

	case (TControllerButtons)4:	//if right button pressed
		if(!LCD_buttonsPressed[0]){
			LCD_buttonsPressed[0] = true;
			if(LCD_currentMenuMode == Loaded){
				LCD_mogoLoaded = !LCD_mogoLoaded;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == Retracting){
				LCD_retracting = !LCD_retracting;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == menuGyro){
				LCD_gyro = !LCD_gyro;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == Constants){
				switch(LCD_currentConstant){
				case driveSubsKP:
					LCD_currentConstant = driveSubsKI;
					LCD_refresh();
					break;
				case driveSubsKI:
					LCD_currentConstant = driveSubsKD;
					LCD_refresh();
					break;
				case driveSubsKD:
					LCD_currentConstant = mogoIntakeSubsKP;
					LCD_refresh();
					break;
				case mogoIntakeSubsKP:
					LCD_currentConstant = mogoIntakeSubsKI;
					LCD_refresh();
					break;
				case mogoIntakeSubsKI:
					LCD_currentConstant = mogoIntakeSubsKD;
					LCD_refresh();
					break;
				case mogoIntakeSubsKD:
					LCD_currentConstant = armSubsKP;
					LCD_refresh();
					break;
				case armSubsKP:
					LCD_currentConstant = armSubsKI;
					LCD_refresh();
					break;
				case armSubsKI:
					LCD_currentConstant = armSubsKD;
					LCD_refresh();
					break;
				case armSubsKD:
					LCD_currentConstant = driveSubsKP;
					LCD_refresh();
					break;
				}
			}
			else if(LCD_currentMenuMode == ManipulateConstants){
				switch(LCD_currentConstant){
				case driveSubsKP:
					if(LCD_mogoLoaded && LCD_gyro) LCD_addToConstant(&PID_KPdriveGyro[1][0]);
					else if(LCD_mogoLoaded && !LCD_gyro) LCD_addToConstant(&PID_KPdrive[1][0]);
					else if(!LCD_mogoLoaded && LCD_gyro) LCD_addToConstant(&PID_KPdriveGyro[0][0]);
					else if(!LCD_mogoLoaded && !LCD_gyro) LCD_addToConstant(&PID_KPdrive[0][0]);
					LCD_refresh();
					break;
				case driveSubsKI:
					if(LCD_mogoLoaded && LCD_gyro) LCD_addToConstant(&PID_KIdriveGyro[1][0]);
					else if(LCD_mogoLoaded && !LCD_gyro) LCD_addToConstant(&PID_KIdrive[1][0]);
					else if(!LCD_mogoLoaded && LCD_gyro) LCD_addToConstant(&PID_KIdriveGyro[0][0]);
					else if(!LCD_mogoLoaded && !LCD_gyro) LCD_addToConstant(&PID_KIdrive[0][0]);
					LCD_refresh();
					break;
				case driveSubsKD:
					if(LCD_mogoLoaded && LCD_gyro) LCD_addToConstant(&PID_KDdriveGyro[1][0], 0.005);
					else if(LCD_mogoLoaded && !LCD_gyro) LCD_addToConstant(&PID_KDdrive[1][0], 0.005);
					else if(!LCD_mogoLoaded && LCD_gyro) LCD_addToConstant(&PID_KDdriveGyro[0][0], 0.005);
					else if(!LCD_mogoLoaded && !LCD_gyro) LCD_addToConstant(&PID_KDdrive[0][0], 0.005);
					LCD_refresh();
					break;

				case mogoIntakeSubsKP:
					if(LCD_mogoLoaded && LCD_retracting) LCD_addToConstant(&PID_KPmobileGoalIntake[1][1]);
					else if(LCD_mogoLoaded && !LCD_retracting) LCD_addToConstant(&PID_KPmobileGoalIntake[1][0]);
					else if(!LCD_mogoLoaded && LCD_retracting) LCD_addToConstant(&PID_KPmobileGoalIntake[0][1]);
					else if(!LCD_mogoLoaded && !LCD_retracting) LCD_addToConstant(&PID_KPmobileGoalIntake[0][0]);
					LCD_refresh();
					break;
				case mogoIntakeSubsKI:
					if(LCD_mogoLoaded && LCD_retracting) LCD_addToConstant(&PID_KImobileGoalIntake[1][1]);
					else if(LCD_mogoLoaded && !LCD_retracting) LCD_addToConstant(&PID_KImobileGoalIntake[1][0]);
					else if(!LCD_mogoLoaded && LCD_retracting) LCD_addToConstant(&PID_KImobileGoalIntake[0][1]);
					else if(!LCD_mogoLoaded && !LCD_retracting) LCD_addToConstant(&PID_KImobileGoalIntake[0][0]);
					LCD_refresh();
					break;
				case mogoIntakeSubsKD:
					if(LCD_mogoLoaded && LCD_retracting) LCD_addToConstant(&PID_KDmobileGoalIntake[1][1], 0.005);
					else if(LCD_mogoLoaded && !LCD_retracting) LCD_addToConstant(&PID_KDmobileGoalIntake[1][0], 0.005);
					else if(!LCD_mogoLoaded && LCD_retracting) LCD_addToConstant(&PID_KDmobileGoalIntake[0][1], 0.005);
					else if(!LCD_mogoLoaded && !LCD_retracting) LCD_addToConstant(&PID_KDmobileGoalIntake[0][0], 0.005);
					LCD_refresh();
					break;

				case armSubsKP:
					if(LCD_mogoLoaded) LCD_addToConstant(&PID_KParm[1]);
					else if(!LCD_mogoLoaded) LCD_addToConstant(&PID_KParm[0]);
					LCD_refresh();
					break;
				case armSubsKI:
					if(LCD_mogoLoaded) LCD_addToConstant(&PID_KIarm[1]);
					else if(!LCD_mogoLoaded) LCD_addToConstant(&PID_KIarm[0]);
					LCD_refresh();
					break;
				case armSubsKD:
					if(LCD_mogoLoaded) LCD_addToConstant(&PID_KDarm[1], 0.005);
					else if(!LCD_mogoLoaded) LCD_addToConstant(&PID_KDarm[0], 0.005);
					LCD_refresh();
					break;
				}
			}
		}
		break;
	default:
		LCD_buttonsPressed[0] = false;
		LCD_buttonsPressed[1] = false;
		LCD_buttonsPressed[2] = false;
	}
}

#endif
#endif
