/*
 * DogControl.c
 *
 *  Created on: Jul 12, 2023
 *      Author: AndrewMajors
 *
 *
 * This file contains functions and parameters that control the dog at a high level
 * uses serial coms, leg control, and sensor data
 */

#include "DogControl_cfg.h"
#include "../PiSerialComs/PiSerialComs.h"
#include "../LegControl/LegControl.h"


int DOG_CONT_ProcessCommand(PSC_CMD cmd);

void DOG_CONT_walkingGait_1(LEG_CONT_Leg leg, float start, float distance, float percentage, VECT_3D offset);
void DOG_CONT_walkingGait_2(LEG_CONT_Leg leg, float start, float distance, float percentage, VECT_3D offset, float angle);
