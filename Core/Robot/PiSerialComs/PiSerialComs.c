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


char PSC_INPUT_BUFFER[PSC_BUFFER_SIZE] = {};
char PSC_MESSAGE[PSC_BUFFER_SIZE] = { };
volatile char PSC_OUTPUT_BUFFER[PSC_BUFFER_SIZE] = {};
int PSC_BUFFER_INDEX = 0;
int PSC_MSG_LEN = 0;

volatile int PSC_NEW_DATA_FROM_BOARD = 0;
volatile int PSC_g_outputDataReady = 0;
volatile int PSC_g_outputDataLen = 0;

void PSC_InitBuffers(){
	for(int i=0; i<PSC_BUFFER_SIZE; i++){
		PSC_INPUT_BUFFER[i]='0';
		PSC_MESSAGE[i]='0';
		PSC_OUTPUT_BUFFER[i] = '0';
	}
}

int PSC_checkBuffer() {
	int i=0;
	for (i = 0; i < PSC_BUFFER_INDEX; i++) {
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

int PSC_ProcessCommand(PSC_CMD cmd){
	int getOrSet = 0; // 0 for get 1 for set;
	int messageLen = 0;
	char response[20];
	if(cmd.action == SETPARAM){
		messageLen = sprintf(response, "<%s set>",PARAM_STRING[cmd.param]);
		PSC_SendToOutputBuffer( response, messageLen);
		getOrSet = 1;
	}
	switch(cmd.param){
		case SPEED:
			if(getOrSet){LEG_CONT_g_walkMaxTime = cmd.vals[0];}
			else{
				messageLen = sprintf(response, "<%0.2f;>",LEG_CONT_g_walkMaxTime);
				PSC_SendToOutputBuffer( response, messageLen);
			}
			break;
		case DIST:
			if(getOrSet){LEG_CONT_g_walkDistance = cmd.vals[0];}
			else{
				messageLen = sprintf(response, "<%0.2f;>",LEG_CONT_g_walkDistance);
				PSC_SendToOutputBuffer(response, messageLen);
			}
			break;
		case HEIGHT:
			if(getOrSet){LEG_CONT_g_walkHeight = cmd.vals[0];}
			else{
				messageLen = sprintf(response, "<%0.2f;>",LEG_CONT_g_walkHeight);
				PSC_SendToOutputBuffer( response, messageLen);
			}
			break;
		case DIR:
			if(getOrSet){LEG_CONT_g_walkDirection = cmd.vals[0];}
			else{
				messageLen = sprintf(response, "<%0.2f;>",LEG_CONT_g_walkDirection);
				PSC_SendToOutputBuffer( response, messageLen);
			}
			break;
		case OPLO:
			if(getOrSet){
				LEG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].x = cmd.vals[1];
				LEG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].y = cmd.vals[2];
			}else{
				messageLen = sprintf(response, "<%0.2f,%0.2f;>",LEG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].x, LEG_CONT_g_walkOpenLoopOffsets[(int)cmd.vals[0]].y);
				PSC_SendToOutputBuffer( response, messageLen);
			}
			break;
		case STOF:
			if(getOrSet){
				LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].x = cmd.vals[1];
				LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].y = cmd.vals[2];
			}else{
				messageLen = sprintf(response, "<%0.2f,%0.2f;>",LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].x, LEG_CONT_g_walkStartOffsets[(int)cmd.vals[0]].y);
				PSC_SendToOutputBuffer( response, messageLen);
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
				PSC_SendToOutputBuffer( response, messageLen);
			}
			break;
		default:
			return 0;
	}
	return 1;
}

int PSC_InterpretCommand(char msg[], int size) {

	char tokens[10][10] = {};
	int tokenSizes [11] = {};
	char *endptr;
	int tokensFound = 0;
	int numTokens =0;
	float floatValue = 0;
	int tokenTemp = -1;
	int tokenStart = 0;
	int tokenEnd = 0;

	PSC_CMD cmd = {};
	while(!tokensFound){
		tokenEnd = PSC_FindNextToken(msg, tokens[numTokens], tokenStart, size);
		if(tokenEnd){
			if(tokenEnd == size){
				tokensFound = 1;
				tokenSizes[numTokens] = tokenEnd-tokenStart;
				numTokens+=1;
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
	if(tokenTemp>=0){
		cmd.action = tokenTemp;
	}else{
		return 0;
	}
	tokenTemp = PSC_EvalParam(tokens[1], tokenSizes[1]);
	if(tokenTemp>=0){
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
	PSC_ProcessCommand(cmd);
	return 1;
}

int PSC_FindNextToken(char str[], char token[], int start, int len){
	int i =0;
	for(i=start; i<=len; i++){

		if(str[i] == ',' || str[i] == ';'){
			return i;
		}
		token[i-start] = str[i];
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

int PSC_SendToOutputBuffer(char msg[], int len){
	if(len > PSC_BUFFER_SIZE){
		return-1;
	}
	for(int i=0; i<len; i++){
		PSC_OUTPUT_BUFFER[i] = msg[i];
	}
	PSC_g_outputDataReady = 1;
	PSC_g_outputDataLen = len;
	return 1;
}

int PSC_ClearOutBuffer(){

	for(int i=0; i<PSC_BUFFER_SIZE; i++){
		PSC_OUTPUT_BUFFER[i] = '0';
	}
	PSC_g_outputDataReady = 0;
	PSC_g_outputDataLen = 0;
	return 1;
}

