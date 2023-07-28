/*
 * DogControl_cgg.c
 *
 *  Created on: Jul 28, 2023
 *      Author: AndrewMajors
 */

#include "DogControl_cfg.h"


volatile float DOG_CONT_g_walkMaxTime = 10000;
volatile float DOG_CONT_g_walkDistance = 1;

volatile float DOG_CONT_g_walkOpenLoopOffsetX = .25;
volatile float DOG_CONT_g_walkOpenLoopOffsetY = .5;
volatile float DOG_CONT_g_walkHeight = 1.5;
volatile float DOG_CONT_g_walkDirection = 0;

//(+x,-y)->(-x,-y)
//(-x,-y)->(-x,+y)
//(-x,+y)->(+x,+y)
//(+x,+y)->(+x,-y)
volatile VECT_3D DOG_CONT_g_walkOpenLoopOffset1 = { .4, -.0, 0};
volatile VECT_3D DOG_CONT_g_walkOpenLoopOffset2 = {-.4, -.0, 0};
volatile VECT_3D DOG_CONT_g_walkOpenLoopOffset3 = {-.4,  .5, 0};
volatile VECT_3D DOG_CONT_g_walkOpenLoopOffset4 = { .4,  .5, 0};

volatile  VECT_3D DOG_CONT_g_walkOpenLoopOffsets[4] = {};
