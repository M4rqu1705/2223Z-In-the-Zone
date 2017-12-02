#ifndef COMMONS_H_
#define COMMONS_H_

#include <API.h>

const signed char JOYS_DRIVE_F = 3;
const signed char JOYS_DRIVE_S = 1;

const signed char JOYS_DRIVE_INVERT_GROUP = 8;
const signed char JOYS_DRIVE_INVERT_BUTTON = JOY_RIGHT;

const signed char JOYS_MOGO_RETRACT_GROUP = 6;
const signed char JOYS_MOGO_RETRACT_BUTTON = JOY_DOWN;
const signed char JOYS_MOGO_EXTEND_GROUP = 6;
const signed char JOYS_MOGO_EXTEND_BUTTON = JOY_UP;

const signed char MOTOR_DRIVE_LB = 1;
const signed char MOTOR_DRIVE_LF = 5;
const signed char MOTOR_DRIVE_RF = 6;
const signed char MOTOR_DRIVE_RB = 10;

const signed char MOTOR_MOGO_L = 4;
const signed char MOTOR_MOGO_R = 7;

const signed char SENSOR_MOGO_POT = 4;

const signed char MOGO_MOTOR_SPEED = 127;
const signed short MOGO_POT_RETRACTED = 1265;
const signed short MOGO_POT_EXTENDED = 3100;

#endif
