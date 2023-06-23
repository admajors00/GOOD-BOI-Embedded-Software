/*
 * LegControl.c
 *
 *  Created on: Feb 19, 2021
 *      Author: Troll
 */


#include "LegControl.h"
#include "LegControl_cfg.h"
#include <math.h>
#include "../ECUAL/SERVO.h"
#include "cmsis_os.h"
//#include "main.h"
//#include "../Inc/mutexs.h"

#define PI 3.14159265358979323846
#define RAD_TO_DEG 57.2958
#define DEG_TO_RAD .0175

 float LEG_CONT_servoAngles[12]={};
 float LEG_CONT_servoOffsets[12]={};
 volatile float LEG_CONT_legPositions[4][3]={};
 volatile int LEG_CONT_movingForward=0;

extern  osMutexId_t servoAngleMutexHandle;
void LEG_CONT_initServos(){
	for(int i=0; i<SERVO_NUM; i++){
		SERVO_Init(i);
	}
}

float f(float x) {
	return sqrt(pow(stepDist, 2) - x * x) -height;
}
float g(float x) {
	return sqrt(pow(stepDist, 2) - x * x) -height;
}
float h(float x) {
	return -10 * ((x + .3) * (x + .3)) - .85;
}
float semi_circle_step(float x, float distance, float height){
	return(height/distance)*sqrt(pow(distance,2)- pow(x-distance,2));
}


void LEG_CONT_walkingGait_1(LEG_CONT_Leg leg, float start, float distance, float percentage, float xoffest, float yoffset){	
	float pos  = start - (percentage* distance);
	if(pos > distance /8){
		LEG_CONT_setPosXYZ(leg, xoffest, pos + yoffset ,height);
	}else if(pos <= distance/8 && pos>0){
		pos = 8* ((distance /8 )-pos);
		LEG_CONT_setPosXYZ(leg, xoffest, pos + yoffset ,semi_circle_step(pos,distance, height/2));
	}else if(pos < 0){
		LEG_CONT_setPosXYZ(leg, xoffest, distance + pos + yoffset,height);
	}
}
void LEG_CONT_singleStep_1(LEG_CONT_Leg leg, float angle, float distance) {
	float y = distance * cos(DEG_TO_RAD * angle);
	float x = distance * sin(DEG_TO_RAD * angle);
	for (float i = -distance; i <= distance; i += step_) {
		LEG_CONT_setPosXYZ(leg, i * x, i * y, g(sqrt(pow(i * x, 2) + pow(i * y, 2))));
	}
}


void LEG_CONT_singleStep_2(LEG_CONT_Leg leg, float angle, float stepDist) {
	float y = stepDist * cos(DEG_TO_RAD * angle);
	float x = stepDist * sin(DEG_TO_RAD * angle);
	for (float i = -1; i < 1; i += step_) {
		LEG_CONT_setPosXYZ(leg, i * x, i * y, f(sqrt(pow(i * x, 2) + pow(i * y, 2))));
	}
}

void LEG_CONT_singleStep(LEG_CONT_Leg *leg, float angle, float distance) {
	float y = distance * cos(DEG_TO_RAD * angle);
	float x = distance * sin(DEG_TO_RAD * angle);
	leg->isStepping = 1;
	for (float i = -distance; i <= distance; i += step_) {
		LEG_CONT_setPosXYZ(*leg, (i * x)+(.5*!leg->isLeft)-(.5*leg->isLeft), (i * y)+(.2*leg->isFront)-(.2*!leg->isFront), g(sqrt(pow(i * x, 2) + pow(i * y, 2))));
	}
	leg->isStepping = 0;
}

void LEG_CONT_singleStepSimultaniusLegs(LEG_CONT_Leg *leg1, LEG_CONT_Leg *leg2, float angle, float distance) {
	float y = distance * cos(DEG_TO_RAD * angle);
	float x = distance * sin(DEG_TO_RAD * angle);
	leg1->isStepping = 1;
	leg2->isStepping = 1;
	for (float i = -distance; i <= distance; i += step_) {
		LEG_CONT_setPosXYZ(*leg1, (i * x)+(.5*!leg1->isLeft)-(.5*leg1->isLeft), (i * y)+(.2*leg1->isFront)-(.2*!leg1->isFront), g(sqrt(pow(i * x, 2) + pow(i * y, 2))));

		LEG_CONT_setPosXYZ(*leg2, (i * x)+(.5*!leg2->isLeft)-(.5*leg2->isLeft), (i * y)+(.2*leg2->isFront)-(.2*!leg2->isFront), g(sqrt(pow(i * x, 2) + pow(i * y, 2))));
	}
	leg1->isStepping = 0;
	leg2->isStepping = 0;
}


void LEG_CONT_setPosXYZ(LEG_CONT_Leg leg, float x, float y, float z) {
	LEG_CONT_legPositions[leg.legNum][0] = x;
	LEG_CONT_legPositions[leg.legNum][1] = y;
	LEG_CONT_legPositions[leg.legNum][2] = z;

	float tb, t1, ta;
	float d = sqrt(x * x + z * z);
	//float ta = atan(x / z) * (180.0 / PI);
	if (leg.isLeft) {
		tb = acos(HIP_OFFSET / d) *RAD_TO_DEG;
		ta = atan(x / z) * RAD_TO_DEG;
		t1 = -90.0 - ta - tb;

	} else {
		tb = acos(-HIP_OFFSET / d) * RAD_TO_DEG;
		ta = atan(x / z) * RAD_TO_DEG;
		t1 = -90.0 - ta - tb;

	}
	//float t1 = -90.0 - ta - tb;
	float x0 = HIP_OFFSET * (cos(t1 * DEG_TO_RAD));
	float z0 = HIP_OFFSET * (sin(t1 * DEG_TO_RAD));
	float D = sqrt((x0 - x) * (x0 - x) + (z0 - z) * (z0 - z));
	//LEG_CONT_writeLegT1T2T3(leg, -t1-90, y, D);

	LEG_CONT_setPosYZ(leg, y, D);
	LEG_CONT_writeLegT1(leg, -t1 - 90);
	//  float Dadj = sqrt(4*(l*l) - D*D);
	//  float t3 = acos(1-((D*D)/2*(l*l)));
	//  float q  = (180-t3)/2;
	//  float t2 = atan(y/D) - q;
	//  LEG_CONT_writeLeg(leg, t1, t2, t3);
	//  return 0;
}
void LEG_CONT_setPosXYZForOffset(LEG_CONT_Leg leg, float x, float y, float z, float xOffset, float yOffset, float zOffset) {
	LEG_CONT_legPositions[leg.legNum][0] = x;
	LEG_CONT_legPositions[leg.legNum][1] = y;
	LEG_CONT_legPositions[leg.legNum][2] = z;

	x+=xOffset;
	y+=yOffset;
	z+=zOffset;
	float tb, t1, ta;
	float d = sqrt(x * x + z * z);
	//float ta = atan(x / z) * (180.0 / PI);
	if (leg.isLeft) {
		tb = acos(HIP_OFFSET / d) *RAD_TO_DEG;
		ta = atan(x / z) * RAD_TO_DEG;
		t1 = -90.0 - ta - tb;

	} else {
		tb = acos(-HIP_OFFSET / d) * RAD_TO_DEG;
		ta = atan(x / z) * RAD_TO_DEG;
		t1 = -90.0 - ta - tb;

	}
	//float t1 = -90.0 - ta - tb;
	float x0 = HIP_OFFSET * (cos(t1 * DEG_TO_RAD));
	float z0 = HIP_OFFSET * (sin(t1 * DEG_TO_RAD));
	float D = sqrt((x0 - x) * (x0 - x) + (z0 - z) * (z0 - z));
	//LEG_CONT_writeLegT1T2T3(leg, -t1-90, y, D);

	LEG_CONT_setPosYZ(leg, y, D);
	LEG_CONT_writeLegT1(leg, -t1 - 90);
	//  float Dadj = sqrt(4*(l*l) - D*D);
	//  float t3 = acos(1-((D*D)/2*(l*l)));
	//  float q  = (180-t3)/2;
	//  float t2 = atan(y/D) - q;
	//  LEG_CONT_writeLeg(leg, t1, t2, t3);
	//  return 0;
}
void LEG_CONT_setPosYZ(LEG_CONT_Leg leg, float y, float z) {
	float r, t;
	r = sqrt((y * y) + (z * z));
	if(leg.isLeft){
		t = atan(-y / z) * RAD_TO_DEG;
	}else{
		t = atan(-y / z) * RAD_TO_DEG;
	}
	LEG_CONT_setPosRT(leg, r, t);
}

void LEG_CONT_setPosRT(LEG_CONT_Leg leg, float r, float t) {
	float phi, q, l1, l2;
	r = sqrt((4 * l * l) - (r * r));
	phi = acos(1 - ((r * r) / (2 * l * l)))* RAD_TO_DEG;
	if (leg.isLeft) {
		t=-t;
		phi = phi;
		q = (180 - phi) / 2;
		l1 = 180-t - q;
		l2 = phi;
	}else{
		phi = phi;
		t = -t;
		q = (180 - phi) / 2;
		l1 = 180-t - q;
		l2 = phi;
	}

	LEG_CONT_writeLegT2T3(leg, l1, l2);
}


void LEG_CONT_writeLegT1T2T3(LEG_CONT_Leg leg, float t1, float t2, float t3) {
	if(!IS_RTOS_USED){
		if (leg.isFront == 1) {
			SERVO_MoveTo(leg.swivle, 180.0 - t1 + leg.s_cal);
		} else {
			SERVO_MoveTo(leg.swivle, t1 + leg.s_cal);
		}

		if (leg.isLeft == 1) {
			SERVO_MoveTo(leg.hip, (180.0 - t2 + leg.h_cal) * 2.0 / 3.0);
			SERVO_MoveTo(leg.knee, (180.0 - t3 + leg.k_cal) * 2.0 / 3.0);
		} else {
			SERVO_MoveTo(leg.hip, (t2 + leg.h_cal) * 2.0 / 3.0);
			SERVO_MoveTo(leg.knee, (t3 + leg.k_cal) * 2.0 / 3.0);
		}
	}else{
		if(osMutexAcquire(servoAngleMutexHandle, 0) == osOK){
			if (leg.isFront == 1) {
				LEG_CONT_servoAngles[leg.swivle] = 180.0 - t1 + leg.s_cal;
			} else {
				LEG_CONT_servoAngles[leg.swivle]= t1 + leg.s_cal;
			}

			if (leg.isLeft == 1) {
				LEG_CONT_servoAngles[leg.hip] = (180.0 - t2 + leg.h_cal) * 2.0 / 3.0;
				LEG_CONT_servoAngles[leg.knee]= (180.0 - t3 + leg.k_cal) * 2.0 / 3.0;
			} else {
				LEG_CONT_servoAngles[leg.hip]= (t2 + leg.h_cal) * 2.0 / 3.0;
				LEG_CONT_servoAngles[leg.knee]= (t3 + leg.k_cal) * 2.0 / 3.0;
			}
			osMutexRelease(servoAngleMutexHandle);
		}
	}
}
void LEG_CONT_writeLegT2T3(LEG_CONT_Leg leg, float t2, float t3) {
	//leg.swivle.write(90);
	if(!IS_RTOS_USED){
		if (leg.isLeft == 1) {
			SERVO_MoveTo(leg.hip, ((180-t2 + leg.h_cal) * 2.0 / 3.0));
			SERVO_MoveTo(leg.knee, ((180-t3 + leg.k_cal) * 2.0 / 3.0));
		} else {
			SERVO_MoveTo(leg.hip, (t2 + leg.h_cal) * 2.0 / 3.0);
			SERVO_MoveTo(leg.knee, (t3 + leg.k_cal) * 2.0 / 3.0);
		}
	}else{
		if(osMutexAcquire(servoAngleMutexHandle, 0) == osOK){
			if (leg.isLeft == 1) {
				LEG_CONT_servoAngles[leg.hip] = (180 - t2 + leg.h_cal) * 2.0 / 3.0;
				LEG_CONT_servoAngles[leg.knee]= (180 - t3 + leg.k_cal) * 2.0 / 3.0;
			} else {
				LEG_CONT_servoAngles[leg.hip]= (t2 + leg.h_cal) * 2.0 / 3.0;
				LEG_CONT_servoAngles[leg.knee]= (t3 + leg.k_cal) * 2.0 / 3.0;
			}
			osMutexRelease(servoAngleMutexHandle);
		}
	}
}
void LEG_CONT_writeLegT1(LEG_CONT_Leg leg, float t1) {
	if(!IS_RTOS_USED){

		if (leg.isFront == 1) {
			SERVO_MoveTo(leg.swivle, 180 - t1 + leg.s_cal);
		} else {
			SERVO_MoveTo(leg.swivle, t1 + leg.s_cal);
		}
	}else{
		if(osMutexAcquire(servoAngleMutexHandle, 0) == osOK){
			if (leg.isFront == 1) {
				LEG_CONT_servoAngles[leg.swivle]= 180 - t1 + leg.s_cal;
			} else {
				LEG_CONT_servoAngles[leg.swivle]= t1 + leg.s_cal;
			}
			osMutexRelease(servoAngleMutexHandle);
		}
	}
}

void LEG_CONT_getOffsetsXYZ(LEG_CONT_Leg leg, float x, float y, float z) {
	float tb, t1, t2, t3, ta, x0, z0, d, D, r, t, phi, q, l1, l2;
	d = sqrt(x * x + z * z);
	//float ta = atan(x / z) * (180.0 / PI);
	if (leg.isLeft) {
		tb = acos(HIP_OFFSET / d) *RAD_TO_DEG;
		ta = atan(x / z) * RAD_TO_DEG;
		t1 = -90.0 - ta - tb;

	} else {
		tb = acos(-HIP_OFFSET / d) * RAD_TO_DEG;
		ta = atan(x / z) * RAD_TO_DEG;
		t1 = -90.0 - ta - tb;

	}
	//float t1 = -90.0 - ta - tb;
	x0 = HIP_OFFSET * (cos(t1 * DEG_TO_RAD));
	z0 = HIP_OFFSET * (sin(t1 * DEG_TO_RAD));
	D = sqrt((x0 - x) * (x0 - x) + (z0 - z) * (z0 - z));
	//LEG_CONT_writeLegT1T2T3(leg, -t1-90, y, D);

	z = D;
	r = sqrt((y * y) + (z * z));
	if(leg.isLeft){
		t = atan(-y / z) * RAD_TO_DEG;
	}else{
		t = atan(-y / z) * RAD_TO_DEG;
	}
	r = sqrt((4 * l * l) - (r * r));
	phi = acos(1 - ((r * r) / (2 * l * l)))* RAD_TO_DEG;
	if (leg.isLeft) {
		t=-t;
		phi = phi;
		q = (180 - phi) / 2;
		l1 = 180-t - q;
		l2 = phi;
	}else{
		phi = phi;
		t = -t;
		q = (180 - phi) / 2;
		l1 = 180-t - q;
		l2 = phi;
	}

	t2 = l1;
	t3 = l2;
	if (leg.isLeft == 1) {
		LEG_CONT_servoOffsets[leg.hip] = ((180 - t2 + leg.h_cal) * 2.0 / 3.0) - LEG_CONT_servoAngles[leg.hip];
		LEG_CONT_servoOffsets[leg.knee]= ((180 - t3 + leg.k_cal) * 2.0 / 3.0) -  LEG_CONT_servoAngles[leg.knee];
	} else {
		LEG_CONT_servoOffsets[leg.hip] = ((t2 + leg.h_cal) * 2.0 / 3.0) - LEG_CONT_servoAngles[leg.hip];
		LEG_CONT_servoOffsets[leg.knee]= ((t3 + leg.k_cal) * 2.0 / 3.0)- LEG_CONT_servoAngles[leg.knee];
	}
	t1 = -t1-90;
	if (leg.isFront == 1) {
		LEG_CONT_servoOffsets[leg.swivle] = (180 - t1 + leg.s_cal) - LEG_CONT_servoAngles[leg.swivle];
	} else {
		LEG_CONT_servoOffsets[leg.swivle] = (t1 + leg.s_cal) - LEG_CONT_servoAngles[leg.swivle];
	}

}



