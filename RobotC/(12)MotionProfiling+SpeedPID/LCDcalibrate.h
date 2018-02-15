#ifndef fullRobotTestLCD.h
#define fullRobotTestLCD.h

#pragma systemfile

#ifdef META_usingLCD    //Only include code if META_usingLCD is defined in the constants.h file
bool LCD_retracting = false, LCD_mogoLoaded = false;
enum LCD_menuMode{Retracting, Loaded, Constants, ManipulateConstants};
enum LCD_PIDconstantsEnum {driveSubsKP, driveSubsKI, driveSubsKD,
	mogoIntakeSubsKP, mogoIntakeSubsKI, mogoIntakeSubsKD,
	armSubsKP, armSubsKI, armSubsKD};

LCD_menuMode LCD_currentMenuMode = Retracting;
LCD_PIDconstantsEnum LCD_currentConstant = driveSubsKP;

bool LCD_buttonsPressed[3];
string LCD_output;    //Declare variable in which the formatted string containing a variable LCD output will be stored

void LCD_init();
void LCD_refresh();
void LCD_calibrate();
void LCD_addToConstant(float &constant);
void LCD_substractToConstant(float &constant);
void LCD_addToConstant(float &constant, float add);
void LCD_substractToConstant(float &constant, float substract);

void LCD_init(){
	clearLCDLine(0);                  //Clear LCD
	clearLCDLine(1);                  //Clear LCD
	bLCDBacklight = META_lcdBacklight;    //Turn backlight on or off based on META_LCDbacklight constant

	LCD_retracting = false;
	LCD_mogoLoaded = false;
	LCD_currentConstant = driveSubsKP;

	//None of the LCD buttons have been previously pressed
	LCD_buttonsPressed[0] = false;
	LCD_buttonsPressed[1] = false;
	LCD_buttonsPressed[2] = false;

}

void LCD_refresh(){
	if(LCD_currentMenuMode == Retracting){
		displayLCDCenteredString(0, "Retracting");
		if(LCD_retracting){
			LCD_output = "True";
		}
		else{
			LCD_output = "False";
		}
		displayLCDCenteredString(1, LCD_output);
	}
	else if(LCD_currentMenuMode == Loaded){
		displayLCDCenteredString(0, "Loaded");
		if(LCD_mogoLoaded){
			LCD_output = "True";
		}
		else{
			LCD_output = "False";
		}
		displayLCDCenteredString(1, LCD_output);
	}
	else if(LCD_currentMenuMode == Constants || LCD_currentMenuMode == ManipulateConstants){
		switch(LCD_currentConstant){
		case driveSubsKP:
			if(LCD_mogoLoaded){
				displayLCDCenteredString(0, "Drv KP Loaded");
				sprintf(LCD_output, "%1.3f", PID_KPdriveLoaded);
				displayLCDCenteredString(1, LCD_output);
			}
			else{
				displayLCDCenteredString(0, "Drv KP UnLded");
				sprintf(LCD_output, "%1.3f", PID_KPdriveUnloaded);
				displayLCDCenteredString(1, LCD_output);
			}
			break;
		case driveSubsKI:
			if(LCD_mogoLoaded){
				displayLCDCenteredString(0, "Drv KI Loaded");
				sprintf(LCD_output, "%1.3f", PID_KIdriveLoaded);
				displayLCDCenteredString(1, LCD_output);
			}
			else{
				displayLCDCenteredString(0, "Drv KI UnLded");
				sprintf(LCD_output, "%1.3f", PID_KIdriveUnloaded);
				displayLCDCenteredString(1, LCD_output);
			}
			break;
		case driveSubsKD:
			if(LCD_mogoLoaded){
				displayLCDCenteredString(0, "Drv KD Loaded");
				sprintf(LCD_output, "%1.3f", PID_KDdriveLoaded);
				displayLCDCenteredString(1, LCD_output);
			}
			else{
				displayLCDCenteredString(0, "Drv KD UnLded");
				sprintf(LCD_output, "%1.3f", PID_KDdriveUnloaded);
				displayLCDCenteredString(1, LCD_output);
			}
			break;
		case mogoIntakeSubsKP:
			if(LCD_retracting){
				if(LCD_mogoLoaded){
					displayLCDCenteredString(0, "MGR KP Loaded");
					sprintf(LCD_output, "%1.3f", PID_KPmobileGoalIntakeLoadedRetract);
					displayLCDCenteredString(1, LCD_output);
				}
				else{
					displayLCDCenteredString(0, "MGR KP UnLded");
					sprintf(LCD_output, "%1.3f", PID_KPmobileGoalIntakeUnloadedRetract);
					displayLCDCenteredString(1, LCD_output);
				}
			}
			else{
				if(LCD_mogoLoaded){
					displayLCDCenteredString(0, "MGE KP Loaded");
					sprintf(LCD_output, "%1.3f", PID_KPmobileGoalIntakeLoadedExtend);
					displayLCDCenteredString(1, LCD_output);
				}
				else{
					displayLCDCenteredString(0, "MGE KP UnLded");
					sprintf(LCD_output, "%1.3f", PID_KPmobileGoalIntakeUnloadedExtend);
					displayLCDCenteredString(1, LCD_output);
				}
			}
			break;
		case mogoIntakeSubsKI:
			if(LCD_retracting){
				if(LCD_mogoLoaded){
					displayLCDCenteredString(0, "MGR KI Loaded");
					sprintf(LCD_output, "%1.3f", PID_KImobileGoalIntakeLoadedRetract);
					displayLCDCenteredString(1, LCD_output);
				}
				else{
					displayLCDCenteredString(0, "MGR KI UnLded");
					sprintf(LCD_output, "%1.3f", PID_KImobileGoalIntakeUnloadedRetract);
					displayLCDCenteredString(1, LCD_output);
				}
			}
			else{
				if(LCD_mogoLoaded){
					displayLCDCenteredString(0, "MGE KI Loaded");
					sprintf(LCD_output, "%1.3f", PID_KImobileGoalIntakeLoadedExtend);
					displayLCDCenteredString(1, LCD_output);
				}
				else{
					displayLCDCenteredString(0, "MGE KI UnLded");
					sprintf(LCD_output, "%1.3f", PID_KImobileGoalIntakeUnloadedExtend);
					displayLCDCenteredString(1, LCD_output);
				}
			}
			break;
		case mogoIntakeSubsKD:
			if(LCD_retracting){
				if(LCD_mogoLoaded){
					displayLCDCenteredString(0, "MGR KD Loaded");
					sprintf(LCD_output, "%1.3f", PID_KDmobileGoalIntakeLoadedRetract);
					displayLCDCenteredString(1, LCD_output);
				}
				else{
					displayLCDCenteredString(0, "MGR KD UnLded");
					sprintf(LCD_output, "%1.3f", PID_KDmobileGoalIntakeUnloadedRetract);
					displayLCDCenteredString(1, LCD_output);
				}
			}
			else{
				if(LCD_mogoLoaded){
					displayLCDCenteredString(0, "MGE KD Loaded");
					sprintf(LCD_output, "%1.3f", PID_KDmobileGoalIntakeLoadedExtend);
					displayLCDCenteredString(1, LCD_output);
				}
				else{
					displayLCDCenteredString(0, "MGE KD UnLded");
					sprintf(LCD_output, "%1.3f", PID_KDmobileGoalIntakeUnloadedExtend);
					displayLCDCenteredString(1, LCD_output);
				}
			}
			break;

		case armSubsKP:
			if(LCD_mogoLoaded){
				displayLCDCenteredString(0, "Arm KP Loaded");
				sprintf(LCD_output, "%1.3f", PID_KParmUp);
				displayLCDCenteredString(1, LCD_output);
			}
			else{
				displayLCDCenteredString(0, "Arm KP UnLded");
				sprintf(LCD_output, "%1.3f", PID_KParmDown);
				displayLCDCenteredString(1, LCD_output);
			}
			break;
		case armSubsKI:
			if(LCD_mogoLoaded){
				displayLCDCenteredString(0, "Arm KI Loaded");
				sprintf(LCD_output, "%1.3f", PID_KIarmUp);
				displayLCDCenteredString(1, LCD_output);
			}
			else{
				displayLCDCenteredString(0, "Arm KI UnLded");
				sprintf(LCD_output, "%1.3f", PID_KIarmDown);
				displayLCDCenteredString(1, LCD_output);
			}
			break;
		case armSubsKD:
			if(LCD_mogoLoaded){
				displayLCDCenteredString(0, "Arm KD Loaded");
				sprintf(LCD_output, "%1.3f", PID_KDarmUp);
				displayLCDCenteredString(1, LCD_output);
			}
			else{
				displayLCDCenteredString(0, "Arm KD UnLded");
				sprintf(LCD_output, "%1.3f", PID_KDarmDown);
				displayLCDCenteredString(1, LCD_output);
			}
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
					if(LCD_mogoLoaded){
						LCD_substractToConstant(PID_KPdriveLoaded);
						LCD_refresh();
					}
					else{
						LCD_substractToConstant(PID_KPdriveUnloaded);
						LCD_refresh();
					}
					break;
				case driveSubsKI:
					if(LCD_mogoLoaded){
						LCD_substractToConstant(PID_KIdriveLoaded);
						LCD_refresh();
					}
					else{
						LCD_substractToConstant(PID_KIdriveUnloaded);
						LCD_refresh();
					}
					break;
				case driveSubsKD:
					if(LCD_mogoLoaded){
						LCD_substractToConstant(PID_KDdriveLoaded, 0.01);
						LCD_refresh();
					}
					else{
						LCD_substractToConstant(PID_KDdriveUnloaded, 0.01);
						LCD_refresh();
					}
					break;

				case mogoIntakeSubsKP:
					if(LCD_retracting){
						if(LCD_mogoLoaded){
							LCD_substractToConstant(PID_KPmobileGoalIntakeLoadedRetract);
							LCD_refresh();
						}
						else{
							LCD_substractToConstant(PID_KPmobileGoalIntakeUnloadedRetract);
							LCD_refresh();
						}
					}
					else{
						if(LCD_mogoLoaded){
							LCD_substractToConstant(PID_KPmobileGoalIntakeLoadedExtend);
							LCD_refresh();
						}
						else{
							LCD_substractToConstant(PID_KPmobileGoalIntakeUnloadedExtend);
							LCD_refresh();
						}
					}
					break;
				case mogoIntakeSubsKI:
					if(LCD_retracting){
						if(LCD_mogoLoaded){
							LCD_substractToConstant(PID_KImobileGoalIntakeLoadedRetract);
							LCD_refresh();
						}
						else{
							LCD_substractToConstant(PID_KImobileGoalIntakeUnloadedRetract);
							LCD_refresh();
						}
					}
					else{
						if(LCD_mogoLoaded){
							LCD_substractToConstant(PID_KImobileGoalIntakeLoadedExtend);
							LCD_refresh();
						}
						else{
							LCD_substractToConstant(PID_KImobileGoalIntakeUnloadedExtend);
							LCD_refresh();
						}
					}
					break;
				case mogoIntakeSubsKD:
					if(LCD_retracting){
						if(LCD_mogoLoaded){
							LCD_substractToConstant(PID_KDmobileGoalIntakeLoadedRetract, 0.01);
							LCD_refresh();
						}
						else{
							LCD_substractToConstant(PID_KDmobileGoalIntakeUnloadedRetract, 0.01);
							LCD_refresh();
						}
					}
					else{
						if(LCD_mogoLoaded){
							LCD_substractToConstant(PID_KDmobileGoalIntakeLoadedExtend, 0.01);
							LCD_refresh();
						}
						else{
							LCD_substractToConstant(PID_KDmobileGoalIntakeUnloadedExtend, 0.01);
							LCD_refresh();
						}
					}
					break;
				case armSubsKP:
					if(LCD_mogoLoaded){
						LCD_substractToConstant(PID_KParmUp);
						LCD_refresh();
					}
					else{
						LCD_substractToConstant(PID_KParmDown);
						LCD_refresh();
					}
					break;
				case armSubsKI:
					if(LCD_mogoLoaded){
						LCD_substractToConstant(PID_KIarmUp);
						LCD_refresh();
					}
					else{
						LCD_substractToConstant(PID_KIarmDown);
						LCD_refresh();
					}
					break;
				case armSubsKD:
					if(LCD_mogoLoaded){
						LCD_substractToConstant(PID_KDarmUp, 0.01);
						LCD_refresh();
					}
					else{
						LCD_substractToConstant(PID_KDarmDown, 0.01);
						LCD_refresh();
					}
					break;
				}
			}
		}
		break;

	case (TControllerButtons)4:	//if right button pressed
		if(!LCD_buttonsPressed[2]){
			LCD_buttonsPressed[2] = true;
			if(LCD_currentMenuMode == Loaded){
				LCD_mogoLoaded = !LCD_mogoLoaded;
				LCD_refresh();
			}
			else if(LCD_currentMenuMode == Retracting){
				LCD_retracting = !LCD_retracting;
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
					if(LCD_mogoLoaded){
						LCD_addToConstant(PID_KPdriveLoaded);
						LCD_refresh();
					}
					else{
						LCD_addToConstant(PID_KPdriveUnloaded);
						LCD_refresh();
					}
					break;
				case driveSubsKI:
					if(LCD_mogoLoaded){
						LCD_addToConstant(PID_KIdriveLoaded);
						LCD_refresh();
					}
					else{
						LCD_addToConstant(PID_KIdriveUnloaded);
						LCD_refresh();
					}
					break;
				case driveSubsKD:
					if(LCD_mogoLoaded){
						LCD_addToConstant(PID_KDdriveLoaded, 0.01);
						LCD_refresh();
					}
					else{
						LCD_addToConstant(PID_KDdriveUnloaded, 0.01);
						LCD_refresh();
					}
					break;

				case mogoIntakeSubsKP:
					if(LCD_retracting){
						if(LCD_mogoLoaded){
							LCD_addToConstant(PID_KPmobileGoalIntakeLoadedRetract);
							LCD_refresh();
						}
						else{
							LCD_addToConstant(PID_KPmobileGoalIntakeUnloadedRetract);
							LCD_refresh();
						}
					}
					else{
						if(LCD_mogoLoaded){
							LCD_addToConstant(PID_KPmobileGoalIntakeLoadedExtend);
							LCD_refresh();
						}
						else{
							LCD_addToConstant(PID_KPmobileGoalIntakeUnloadedExtend);
							LCD_refresh();
						}
					}
					break;
				case mogoIntakeSubsKI:
					if(LCD_retracting){
						if(LCD_mogoLoaded){
							LCD_addToConstant(PID_KImobileGoalIntakeLoadedRetract);
							LCD_refresh();
						}
						else{
							LCD_addToConstant(PID_KImobileGoalIntakeUnloadedRetract);
							LCD_refresh();
						}
					}
					else{
						if(LCD_mogoLoaded){
							LCD_addToConstant(PID_KImobileGoalIntakeLoadedExtend);
							LCD_refresh();
						}
						else{
							LCD_addToConstant(PID_KImobileGoalIntakeUnloadedExtend);
							LCD_refresh();
						}
					}
					break;
				case mogoIntakeSubsKD:
					if(LCD_retracting){
						if(LCD_mogoLoaded){
							LCD_addToConstant(PID_KDmobileGoalIntakeLoadedRetract, 0.01);
							LCD_refresh();
						}
						else{
							LCD_addToConstant(PID_KDmobileGoalIntakeUnloadedRetract, 0.01);
							LCD_refresh();
						}
					}
					else{
						if(LCD_mogoLoaded){
							LCD_addToConstant(PID_KDmobileGoalIntakeLoadedExtend, 0.01);
							LCD_refresh();
						}
						else{
							LCD_addToConstant(PID_KDmobileGoalIntakeUnloadedExtend, 0.01);
							LCD_refresh();
						}
					}
					break;
				case armSubsKP:
					if(LCD_mogoLoaded){
						LCD_addToConstant(PID_KParmUp);
						LCD_refresh();
					}
					else{
						LCD_addToConstant(PID_KParmDown);
						LCD_refresh();
					}
					break;
				case armSubsKI:
					if(LCD_mogoLoaded){
						LCD_addToConstant(PID_KIarmUp);
						LCD_refresh();
					}
					else{
						LCD_addToConstant(PID_KIarmDown);
						LCD_refresh();
					}
					break;
				case armSubsKD:
					if(LCD_mogoLoaded){
						LCD_addToConstant(PID_KDarmUp);
						LCD_refresh();
					}
					else{
						LCD_addToConstant(PID_KDarmDown, 0.01);
						LCD_refresh();
					}
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

void LCD_addToConstant(float &constant){
	constant+=0.05;
}

void LCD_substractToConstant(float &constant){
	constant-=0.05;
}

void LCD_addToConstant(float &constant, float add){
	constant += add;
}

void LCD_substractToConstant(float &constant, float substract){
	constant -= substract
}
#endif
#endif
