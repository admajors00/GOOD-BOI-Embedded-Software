/*
 * LegControl_cfg.c
 *
 *  Created on: Mar 5, 2021
 *      Author: Troll
 *
 * Use this file to setup the legs and assign calibration values
 */


#include "LegControl.h"
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

volatile VECT_3D LEG_CONT_g_walkStartOffset1 = { .5, -.5, 0};
volatile VECT_3D LEG_CONT_g_walkStartOffset2 = {-.5, -.5, 0};
volatile VECT_3D LEG_CONT_g_walkStartOffset3 = {.5,  0, 0};
volatile VECT_3D LEG_CONT_g_walkStartOffset4 = { -.5,  0, 0};


const float stepDist = .5;
const float height = 1.5;
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





float path[][2] = {
{ 0.0 ,  0.0 },
{ 0.031 ,  -0.0 },
{ 0.062 ,  -0.0 },
{ 0.093 ,  -0.0 },
{ 0.124 ,  -0.0 },
{ 0.155 ,  -0.0 },
{ 0.186 ,  -0.0 },
{ 0.218 ,  -0.0 },
{ 0.249 ,  -0.0 },
{ 0.28 ,  -0.0 },
{ 0.311 ,  -0.0 },
{ 0.342 ,  -0.0 },
{ 0.373 ,  -0.0 },
{ 0.404 ,  -0.0 },
{ 0.435 ,  -0.0 },
{ 0.466 ,  -0.0 },
{ 0.497 ,  -0.0 },
{ 0.528 ,  -0.0 },
{ 0.559 ,  -0.0 },
{ 0.59 ,  -0.0 },
{ 0.622 ,  -0.0 },
{ 0.653 ,  -0.0 },
{ 0.684 ,  -0.0 },
{ 0.715 ,  -0.0 },
{ 0.746 ,  -0.0 },
{ 0.777 ,  -0.0 },
{ 0.808 ,  -0.0 },
{ 0.839 ,  -0.0 },
{ 0.87 ,  -0.0 },
{ 0.901 ,  -0.0 },
{ 0.932 ,  -0.0 },
{ 0.963 ,  -0.0 },
{ 1.0 ,  0.0 },
{ 1.024 ,  0.02 },
{ 1.047 ,  0.04 },
{ 1.071 ,  0.06 },
{ 1.11 ,  0.093 },
{ 1.088 ,  0.115 },
{ 1.065 ,  0.136 },
{ 1.043 ,  0.158 },
{ 1.021 ,  0.18 },
{ 0.995 ,  0.205 },
{ 0.965 ,  0.213 },
{ 0.935 ,  0.22 },
{ 0.905 ,  0.228 },
{ 0.874 ,  0.235 },
{ 0.844 ,  0.243 },
{ 0.814 ,  0.25 },
{ 0.784 ,  0.258 },
{ 0.754 ,  0.265 },
{ 0.724 ,  0.273 },
{ 0.693 ,  0.28 },
{ 0.663 ,  0.288 },
{ 0.633 ,  0.295 },
{ 0.603 ,  0.303 },
{ 0.573 ,  0.31 },
{ 0.543 ,  0.318 },
{ 0.512 ,  0.325 },
{ 0.482 ,  0.333 },
{ 0.452 ,  0.34 },
{ 0.422 ,  0.348 },
{ 0.392 ,  0.355 },
{ 0.362 ,  0.363 },
{ 0.332 ,  0.37 },
{ 0.301 ,  0.378 },
{ 0.271 ,  0.385 },
{ 0.236 ,  0.394 },
{ 0.205 ,  0.397 },
{ 0.174 ,  0.401 },
{ 0.143 ,  0.404 },
{ 0.112 ,  0.407 },
{ 0.081 ,  0.411 },
{ 0.051 ,  0.414 },
{ 0.02 ,  0.417 },
{ -0.011 ,  0.42 },
{ -0.042 ,  0.424 },
{ -0.073 ,  0.427 },
{ -0.104 ,  0.43 },
{ -0.147 ,  0.435 },
{ -0.168 ,  0.412 },
{ -0.189 ,  0.389 },
{ -0.21 ,  0.366 },
{ -0.231 ,  0.344 },
{ -0.252 ,  0.321 },
{ -0.29 ,  0.28 },
{ -0.277 ,  0.252 },
{ -0.264 ,  0.224 },
{ -0.25 ,  0.196 },
{ -0.237 ,  0.168 },
{ -0.224 ,  0.139 },
{ -0.21 ,  0.11 },
{ -0.182 ,  0.096 },
{ -0.155 ,  0.081 },
{ -0.127 ,  0.067 },
{ -0.1 ,  0.052 },
{ -0.072 ,  0.038 },
{ -0.045 ,  0.023 }};


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
VECT_3D L_1_startPos;
VECT_3D L_2_startPos;
VECT_3D L_3_startPos;
VECT_3D L_4_startPos;



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



