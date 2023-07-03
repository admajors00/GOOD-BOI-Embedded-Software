/*
 * LegControl_cfg.c
 *
 *  Created on: Mar 5, 2021
 *      Author: Troll
 *
 * Use this file to setup the legs and assign calibration values
 */


#include "LegControl_cfg.h"
#include "../Robot/Vectors.h"



volatile float LEG_CONT_g_walkMaxTime = 10000;
volatile float LEG_CONT_g_walkDistance = 1;

volatile float LEG_CONT_g_walkOpenLoopOffsetX = .25;
volatile float LEG_CONT_g_walkOpenLoopOffsetY = .5;
volatile float LEG_CONT_g_walkHeight = 1.5;
volatile float LEG_CONT_g_walkDirection = 0;

//(+x,-y)->(-x,-y)
//(-x,-y)->(-x,+y)
//(-x,+y)->(+x,+y)
//(+x,+y)->(+x,-y)
volatile VECT_3D LEG_CONT_g_walkOpenLoopOffset1 = { .5, -.5, 0};
volatile VECT_3D LEG_CONT_g_walkOpenLoopOffset2 = {-.5, -.5, 0};
volatile VECT_3D LEG_CONT_g_walkOpenLoopOffset3 = {-.5,  .5, 0};
volatile VECT_3D LEG_CONT_g_walkOpenLoopOffset4 = { .5,  .5, 0};

volatile  VECT_3D LEG_CONT_g_walkOpenLoopOffsets[4] = {};

volatile VECT_3D LEG_CONT_g_walkStartOffset1 = { .5, -.5, 0};
volatile VECT_3D LEG_CONT_g_walkStartOffset2 = {-.5, -.5, 0};
volatile VECT_3D LEG_CONT_g_walkStartOffset3 = {.5,  0, 0};
volatile VECT_3D LEG_CONT_g_walkStartOffset4 = { -.5,  0, 0};

volatile  VECT_3D LEG_CONT_g_walkStartOffsets[4] = {};


const float stepDist = .5;
const float height = 1.5;
const float step_ = .001;
const float HIP_OFFSET = .28;
const float angle = 0;
const float l = 1;

//set to 1 if system is using rtos and 0 if not

LEG_CONT_Leg LEG_CONT_CfgParams[4] ={};


LEG_CONT_Leg L_1 = {
  0, s_1, h_1, k_1, 1, 0, 8, -5, 3, 0
};


LEG_CONT_Leg L_2 = {
  1, s_2, h_2, k_2, 0, 0, -7, 2, -3, 0
};


LEG_CONT_Leg L_3 = {
  2, s_3, h_3, k_3, 1, 1, -5, -3, 3, 0
};


LEG_CONT_Leg L_4 = {
  3, s_4, h_4, k_4, 0, 1, 0, 3, -3, 0
};





