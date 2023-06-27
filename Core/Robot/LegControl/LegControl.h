/*
 * LegControl.h
 *
 *  Created on: Feb 19, 2021
 *      Author: Troll
 */

/*
Andrew Majors
This file controls the legs in 3 dimensions

 orientation of legs
 this is mirrored accross the long axis
 the unit of measuerment is the distance from the hip to the knee which should
 equal the distance from the knee to the foot so the maximum reach of a leg is 2

=======================================

  long axis       |+Z front           directions  0
   of robot       |  /+y           for singleStep |+y
      /           | /                             |
     /  inside    |/    outside                   |     +x
    /   ----------+----------            90-------+------- 270
   /    -X       /|        +X                     |
  /             / |                               |
               /  |-Z                             |
         back /-Y                                180

=======================================
*/

#ifndef ROBOT_LEGCONTROL_LEGCONTROL_H_
#define ROBOT_LEGCONTROL_LEGCONTROL_H_
//ss#include "LegControl.h"
//#include "LegControl_cfg.h"
#include "../Robot/Vectors.h"

#define NUM_LEGS 4
#define LEG_CONT_NUM_SERVOS 12


extern float LEG_CONT_servoAngles[12];
extern float LEG_CONT_servoOffsets[12];
extern volatile float LEG_CONT_legPositions[4][3];
extern volatile int LEG_CONT_movingForward;

typedef struct{
  int legNum;
  int swivle;//refers to an index in the SERVO_CfgType SERVO_CfgParam[SERVO_NUM] array in SERVO_cfg.c;
  int hip;   //becasue servos are moved by calling SERVO_MoveTo(SERVO_CfgParam index, angle)
  int knee;
  int isLeft;//0 for right, 1 for left
  int isFront;//0 for back, 1 for front
  int s_cal; //calibration values for each motor in degrees
  int h_cal;
  int k_cal;
  VECT_3D pos;
  VECT_3D startPos;
  volatile int isStepping; //o if not taking a step and 1 if it is

}LEG_CONT_Leg;


/*
 * these 3 functions are used to describe the shape of the path the foot moves on
 */
float f(float x);

float g(float x);

float h(float x);

float semi_circle_step(float x, float distance, float height);

void LEG_CONT_walkingGait_1(LEG_CONT_Leg leg, float start, float distance, float percentage, float xoffest, float yoffset);
void LEG_CONT_walkingGait_2(LEG_CONT_Leg leg, float start, float distance, float percentage, float xoffest, float yoffset, float angle);


/*
 * singleStep moves the foot a specified distance at a certain angle. The path can be described by
 * f(x), g(x) or h(x)
 *
 * there are multiple so that f,g or h, can be used at the same time
 */
void LEG_CONT_singleStep_1(LEG_CONT_Leg leg, float angle, float stepDist);

void LEG_CONT_singleStep_2(LEG_CONT_Leg leg, float angle, float stepDist);

void LEG_CONT_singleStep(LEG_CONT_Leg *leg, float angle, float stepDist);

void LEG_CONT_singleStepSimultaniusLegs(LEG_CONT_Leg *leg1, LEG_CONT_Leg *leg2, float angle, float distance);

/*
 * these three functions set the position of the foot
 *
 * setPosYZ sets the postion of a leg to a coordiante on the yz plane
 * setPosRT sets the position of a leg to a coordinate on the yz plane but uses polar coords as input
 *
 * setPosition xyz sets the postion of a leg to a xyz coord in 3d space
 *
 * these functions call apon the writeLeg functions below by converting xyz coords into angles for the servos
 */

void LEG_CONT_setPosYZ(LEG_CONT_Leg leg, float y, float z) ;

void LEG_CONT_setPosRT(LEG_CONT_Leg leg, float r, float t);

void LEG_CONT_setPosXYZ(LEG_CONT_Leg leg, float x, float y, float z);

void LEG_CONT_setPosXYZForOffset(LEG_CONT_Leg leg, float x, float y, float z, float xOffset, float yoffset, float zOffest);




/*
 * The following functions are the only ones that talk to the SERVO libraries.
 * previous functions call these to set the positions
 *
 * these functions apply the calibration values to the motors
 */

void LEG_CONT_writeLegT1T2T3(LEG_CONT_Leg leg, float t1, float t2, float t3);

void LEG_CONT_writeLegT2T3(LEG_CONT_Leg leg, float t2, float t3);

void LEG_CONT_writeLegT1(LEG_CONT_Leg leg, float t1);

/*initializes the servos that are deined in the SERVO_cfg.c file
*/
void LEG_CONT_initServos();

void LEG_CONT_getOffsetsXYZ(LEG_CONT_Leg leg, float x, float y, float z);

#endif /* ROBOT_LEGCONTROL_LEGCONTROL_H_ */
