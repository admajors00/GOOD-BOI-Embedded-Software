/*
 * PiSerialComs.c
 *
 *  Created on: Sep 12, 2021
 *      Author: Troll
 */


/*
 * List of commands
 * Messaging structure
 * Action,Parameter, <V1,V2,V3>
 *
 * Actions:
 * 		Get
 * 		Set
 *
 * 	Parameters:
 * 		SPEED
 * 		DIST
 * 		HEIGHT
 * 		DIR
 * 		OPLOn	|	Openloop offset values x y
 */
#include "PiSerialComs.h"


char PSC_INPUT_BUFFER[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0 };
char PSC_MESSAGE[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0 };
int PSC_BUFFER_INDEX = 0;
int PSC_MSG_LEN = 0;

volatile int PSC_NEW_DATA_FROM_BOARD = 0;



int PSC_checkBuffer() {
	for (int i = 0; i < PSC_BUFFER_INDEX; i++) {
		if (PSC_INPUT_BUFFER[i] == '<' && !PSC_NEW_DATA_FROM_BOARD) {
			for (int j = i + 1; j < PSC_BUFFER_INDEX; j++) {
				if (PSC_INPUT_BUFFER[j] == '>' && !PSC_NEW_DATA_FROM_BOARD) {
					for (int k = i + 1; k < j; k++) {
						PSC_MESSAGE[k - i - 1] = PSC_INPUT_BUFFER[k];
						PSC_MSG_LEN = k - i - 1;
					}

					//PSC_NEW_DATA_FROM_BOARD = 1;
					PSC_clearBuffer();
					return 1;
					break;
				}
			}
		}

	}
	PSC_MSG_LEN = 0;
	return 0;

}

void PSC_clearBuffer() {

	for (int i = 0; i < PSC_BUFFER_INDEX; i++) {
		PSC_INPUT_BUFFER[i] = '0';
	}
	PSC_BUFFER_INDEX = 0;
}

int PSC_ProcessCommand(PSC_CMD cmd, UART_HandleTypeDef huart){
	int getOrSet = 0; // 0 for get 1 for set;
	int messageLen = 0;
	char response[50];
	if(cmd.action == SETPARAM){
		getOrSet = 1;
	}
	switch(cmd.param){
		case SPEED:
			if(getOrSet){LEG_CONT_g_walkMaxTime = cmd.vals[0];}
			else{
				messageLen = sprintf(response, "<%0.2f;>",LEG_CONT_g_walkMaxTime);
				HAL_UART_Transmit_IT(&huart, (uint8_t*)&response, messageLen);
			}
			break;
		case DIST:
			if(getOrSet){LEG_CONT_g_walkDistance = cmd.vals[0];}
			else{
				messageLen = sprintf(response, "<%0.2f;>",LEG_CONT_g_walkDistance);
				HAL_UART_Transmit_IT(&huart, (uint8_t*)&response, messageLen);
			}
			break;
		case HEIGHT:
			if(getOrSet){LEG_CONT_g_walkHeight = cmd.vals[0];}
			else{
				messageLen = sprintf(response, "<%0.2f;>",LEG_CONT_g_walkHeight);
				HAL_UART_Transmit_IT(&huart, (uint8_t*)&response, messageLen);
			}
			break;
		case DIR:
			if(getOrSet){LEG_CONT_g_walkDirection = cmd.vals[0];}
			else{
				messageLen = sprintf(response, "<%0.2f;>",LEG_CONT_g_walkDirection);
				HAL_UART_Transmit_IT(&huart, (uint8_t*)&response, messageLen);
			}
			break;
		case OPLO:
			if(getOrSet){
				LEG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].x = cmd.vals[1];
				LEG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].y = cmd.vals[2];
			}else{
				messageLen = sprintf(response, "<%0.2f,%0.2f;>",LEG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].x, LEG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].y);
				HAL_UART_Transmit_IT(&huart, (uint8_t*)&response, messageLen);
			}
			break;
		case STOF:
			if(getOrSet){
				LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].x = cmd.vals[1];
				LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].y = cmd.vals[2];
			}else{
				messageLen = sprintf(response, "<%0.2f,%0.2f;>",LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].x, LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].y);
				HAL_UART_Transmit_IT(&huart, (uint8_t*)&response, messageLen);
			}
			break;
		case IMUDATA:
			if(!getOrSet){
				messageLen = sprintf(response, "<%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f;>",
										ADI_IMU_burstReadBufScaled[2],
										ADI_IMU_burstReadBufScaled[3],
										ADI_IMU_burstReadBufScaled[4],
										ADI_IMU_burstReadBufScaled[5],
										ADI_IMU_burstReadBufScaled[6],
										ADI_IMU_burstReadBufScaled[7]);
				HAL_UART_Transmit_IT(&huart, (uint8_t*)&response, messageLen);
			}
			break;
		default:
			return 0;
	}
	return 1;
}

int PSC_InterpretCommand(char msg[], int size,  UART_HandleTypeDef huart) {

	char tokens[10][10];
	int tokenSizes [10];
	char *endptr;
	int tokensFound = 0;
	int numTokens =0;
	float floatValue = 0;
	int tokenTemp = -1;
	int tokenStart = 0;
	int tokenEnd = 0;

	PSC_CMD cmd;
	while(!tokensFound){
		tokenEnd = PSC_FindNextToken(msg, tokens[numTokens], tokenStart, size);
		if(tokenEnd){
			if(tokenEnd == size){
				tokensFound = 1;
			}else{
				tokenSizes[numTokens] = tokenEnd-tokenStart;
				tokenStart = tokenEnd +1;
				numTokens+=1;
			}
		}else{
			return 0;
		}
	}
	tokenTemp = PSC_EvalAction(tokens[0], tokenSizes[0]);
	if(tokenTemp){
		cmd.action = tokenTemp;
	}else{
		return 0;
	}
	tokenTemp = PSC_EvalParam(tokens[1], tokenSizes[1]);
	if(tokenTemp){
		cmd.param = tokenTemp;
	}else{
		return 0;
	}

	if(numTokens > 2){
		for(int i=2; i<numTokens; i++){
			floatValue = strtof(tokens[i], &endptr);
			if (*endptr != '\0') {
				return 0;
			}
			cmd.vals[i-2] = floatValue;
		}
	}
	cmd.numVals = numTokens-2;
	PSC_ProcessCommand(cmd, huart);
	return 1;
}

int PSC_FindNextToken(char str[], char token[], int start, int len){
	int i =0;
	for(i=start; i<len; i++){
		token[i] = str[i];
		if(str[i] == ',' || str[i] == ';'){
			return i;
		}
	}
	return -1;
}

int PSC_EvalAction(char str[],unsigned int len){
	for(int i=0; i<NUM_ACTIONS; i++ ){
		if(strncmp(str, ACTION_STRING[i], len)==0){
			return i;
		}
	}
	return -1;
}

int PSC_EvalParam(char str[],unsigned int len){
	for(int i=0; i<NUM_PARAMS; i++ ){
		if(strncmp(str, PARAM_STRING[i], len)==0){
			return i;
		}
	}
	return -1;
}

