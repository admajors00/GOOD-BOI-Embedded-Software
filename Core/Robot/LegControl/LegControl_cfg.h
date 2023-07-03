/*
 * LegControl_cfg.h
 *
 *  Created on: Mar 5, 2021
 *      Author: Troll
 */

#ifndef ROBOT_LEGCONTROL_LEGCONTROL_CFG_H_
#define ROBOT_LEGCONTROL_LEGCONTROL_CFG_H_

#include "LegControl.h"
#include "../Robot/Vectors.h"

#define LEG_CONT_NUM_SERVOS 12
#define IS_RTOS_USED  1

#define  s_1  0 //swivel
#define  h_1  1 //hip
#define  k_1  2 //knee

#define  s_2  3
#define  h_2  4
#define  k_2  5

#define  s_3  6
#define  h_3  7
#define  k_3  8

#define  s_4  9
#define  h_4  10
#define  k_4  11


extern volatile float LEG_CONT_g_walkMaxTime;
extern volatile float LEG_CONT_g_walkDistance;
extern volatile float LEG_CONT_g_walkOpenLoopOffsetX;
extern volatile float LEG_CONT_g_walkOpenLoopOffsetY;
extern volatile float LEG_CONT_g_walkHeight;
extern volatile float LEG_CONT_g_walkDirection;




extern volatile VECT_3D LEG_CONT_g_walkOpenLoopOffset1 ;
extern volatile VECT_3D LEG_CONT_g_walkOpenLoopOffset2 ;
extern volatile VECT_3D LEG_CONT_g_walkOpenLoopOffset3;
extern volatile VECT_3D LEG_CONT_g_walkOpenLoopOffset4;

extern volatile VECT_3D LEG_CONT_g_walkOpenLoopOffsets[4];

extern volatile VECT_3D LEG_CONT_g_walkStartOffset1;
extern volatile VECT_3D LEG_CONT_g_walkStartOffset2;
extern volatile VECT_3D LEG_CONT_g_walkStartOffset3;
extern volatile VECT_3D LEG_CONT_g_walkStartOffset4;

extern volatile VECT_3D LEG_CONT_g_walkStartOffsets[4];

extern const float stepDist;
extern const float height;
extern const float step_;
extern const float HIP_OFFSET;
extern const float angle;
extern const float l;

extern LEG_CONT_Leg L_1;
extern LEG_CONT_Leg L_2;
extern LEG_CONT_Leg L_3;
extern LEG_CONT_Leg L_4;

extern LEG_CONT_Leg LEG_CONT_CfgParams[4];

#endif /* ROBOT_LEGCONTROL_LEGCONTROL_CFG_H_ */
