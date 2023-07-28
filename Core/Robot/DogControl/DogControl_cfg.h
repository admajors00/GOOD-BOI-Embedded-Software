/*
 * DogControl_cfg.h
 *
 *  Created on: Jul 28, 2023
 *      Author: AndrewMajors
 */

#ifndef ROBOT_DOGCONTROL_DOGCONTROL_CFG_H_
#define ROBOT_DOGCONTROL_DOGCONTROL_CFG_H_

#include "../Robot/Vectors.h"
#include "DogControl.h"

extern volatile float DOG_CONT_g_walkMaxTime;
extern volatile float DOG_CONT_g_walkDistance;
extern volatile float DOG_CONT_g_walkOpenLoopOffsetX;
extern volatile float DOG_CONT_g_walkOpenLoopOffsetY;
extern volatile float DOG_CONT_g_walkHeight;
extern volatile float DOG_CONT_g_walkDirection;




extern volatile VECT_3D DOG_CONT_g_walkOpenLoopOffset1 ;
extern volatile VECT_3D DOG_CONT_g_walkOpenLoopOffset2 ;
extern volatile VECT_3D DOG_CONT_g_walkOpenLoopOffset3;
extern volatile VECT_3D DOG_CONT_g_walkOpenLoopOffset4;

extern volatile VECT_3D DOG_CONT_g_walkOpenLoopOffsets[4];


#endif /* ROBOT_DOGCONTROL_DOGCONTROL_CFG_H_ */
