/*
 * DogControl.c
 *
 *  Created on: Jul 12, 2023
 *      Author: AndrewMajors
 */


#include "DogControl.h"

void DOG_CONT_walkingGait_1(LEG_CONT_Leg leg, float start, float distance, float percentage, VECT_3D offset){
	float pos  = start - (percentage* distance);
	if(pos <0){
		pos += distance;
	}
	if(pos > distance /8){
		LEG_CONT_setPosXYZ(leg, offset.x, pos + offset.y ,DOG_CONT_g_walkHeight);
	}else if(pos <= distance/8 && pos>=0){
		pos = 8* ((distance /8 )-pos);
		LEG_CONT_setPosXYZ(leg, offset.x, pos + offset.y ,semi_circle_step(pos,distance, DOG_CONT_g_walkHeight/2));
	}
}

void DOG_CONT_walkingGait_2(LEG_CONT_Leg leg, float start, float distance, float percentage, VECT_3D offset, float angle){
	float pos  = start - (percentage* distance);
	float posy = pos * cos(DEG_TO_RAD * angle);
	float posx = pos * sin(DEG_TO_RAD * angle);
	if(pos > distance /8){
		LEG_CONT_setPosXYZ(leg,posx+ offset.x, posy + offset.y ,DOG_CONT_g_walkHeight);
	}else if(pos <= distance/8 && pos>0){
		pos = 8* ((distance /8 )-pos);
		LEG_CONT_setPosXYZ(leg, posx +offset.x, posy + offset.y, semi_circle_step(pos,distance, DOG_CONT_g_walkHeight/2));
	}else if(pos < 0){
		LEG_CONT_setPosXYZ(leg,distance +posx+ offset.x, distance + posy + offset.y,DOG_CONT_g_walkHeight);

	}
}

int DOG_CONT_ProcessCommand(PSC_CMD cmd){
	int getOrSet = 0; // 0 for get 1 for set;
	PSC_CMD responseCmd;
	responseCmd.param = cmd.param;
	if(cmd.action == ACTION_ERROR || cmd.param == PARAM_ERROR){
		responseCmd.action = cmd.action;
		responseCmd.param = cmd.param;
		return PSC_SendCmd(responseCmd);
	}
	if(cmd.action == SETPARAM){
		getOrSet = 1;
		responseCmd.action = ACKPARAM;
	}else if (cmd.action == GETPARAM){
		responseCmd.action = SETPARAM;
		responseCmd.param = cmd.param;
	}

	switch(cmd.param){
	case SPEED:
		if(getOrSet){DOG_CONT_g_walkMaxTime = cmd.vals[0];}
		else{
			responseCmd.numVals = 1;
			responseCmd.vals[0] = DOG_CONT_g_walkMaxTime;
		}
		break;
	case DIST:
		if(getOrSet){DOG_CONT_g_walkDistance = cmd.vals[0];}
		else{
			responseCmd.numVals = 1;
			responseCmd.vals[0] =DOG_CONT_g_walkDistance;
		}
		break;
	case HEIGHT:
		if(getOrSet){DOG_CONT_g_walkHeight = cmd.vals[0];}
		else{
			responseCmd.numVals = 1;
			responseCmd.vals[0] =DOG_CONT_g_walkHeight;
		}
		break;
	case DIR:
		if(getOrSet){DOG_CONT_g_walkDirection = cmd.vals[0];}
		else{
			responseCmd.numVals = 1;
			responseCmd.vals[0] =DOG_CONT_g_walkDirection;
		}
		break;
	case OPLO:
		if(getOrSet){
			DOG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].x = cmd.vals[1];
			DOG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].y = cmd.vals[2];
		}else{
			responseCmd.numVals = 3;
			responseCmd.vals[0] = cmd.vals[0];
			responseCmd.vals[1] = DOG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].x;
			responseCmd.vals[2] = DOG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].y;
		}
		break;
	case STOF:
		if(getOrSet){
			LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].x = cmd.vals[1];
			LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].y = cmd.vals[2];
		}else{
			responseCmd.numVals = 3;
			responseCmd.vals[0] = cmd.vals[0];
			responseCmd.vals[1] = LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].x;
			responseCmd.vals[2] = LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].y;
		}
		break;
	case IMUDATA:
		if(!getOrSet){
			responseCmd.numVals = 6;
			responseCmd.vals[0] = ADI_IMU_burstReadBufScaled[2];
			responseCmd.vals[1] = ADI_IMU_burstReadBufScaled[3];
			responseCmd.vals[2] = ADI_IMU_burstReadBufScaled[4];
			responseCmd.vals[3] = ADI_IMU_burstReadBufScaled[5];
			responseCmd.vals[4] = ADI_IMU_burstReadBufScaled[6];
			responseCmd.vals[5] = ADI_IMU_burstReadBufScaled[7];
		}
		break;
	default:
		return 0;

	}


	return PSC_SendCmd(responseCmd);
}
