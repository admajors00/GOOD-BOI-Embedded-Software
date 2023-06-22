/*
 * LegControl_cfg.c
 *
 *  Created on: Mar 5, 2021
 *      Author: Troll
 *
 * Use this file to setup the legs and assign calibration values
 */


#include "LegControl.h"

const float stepDist = .5;
const float height = 1.;
const float step_ = .001;
const float HIP_OFFSET = .28;
const float angle = 0;
const float l = 1;

//set to 1 if system is using rtos and 0 if not

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








//const Leg LegControl_CfgParam[4] = {
//
//		{
//		  s_1, h_1, k_1, 1, 0, 8, 0, 11
//		},
//
//
//		{
//		  s_2, h_2, k_2, 0, 0, -7, 0, 0
//		},
//
//
//		{
//		  s_3, h_3, k_3, 1, 1, -2, 0, 8
//		},
//
//
//		{
//		  s_4, h_4, k_4, 0, 1, 0, 0, 0
//		},
//};


volatile LEG_CONT_Leg L_1 = {
  0, s_1, h_1, k_1, 1, 0, 8, -5, 3, 0
};


volatile LEG_CONT_Leg L_2 = {
  1, s_2, h_2, k_2, 0, 0, -7, 2, -3, 0
};


volatile LEG_CONT_Leg L_3 = {
  2, s_3, h_3, k_3, 1, 1, -5, -3, 3, 0
};


volatile LEG_CONT_Leg L_4 = {
  3, s_4, h_4, k_4, 0, 1, 0, 3, -3, 0
};

//Leg L_1 = {
//  s_1, h_1, k_1, 1, 0, 0, 0, 0
//};
//
//
//Leg L_2 = {
//  s_2, h_2, k_2, 0, 0, 0, 0, 0
//};
//
//
//Leg L_3 = {
//  s_3, h_3, k_3, 1, 1, 0, 0, 0
//};
//
//
//Leg L_4 = {
//  s_4, h_4, k_4, 0, 1, 0, 0, 0
//};



