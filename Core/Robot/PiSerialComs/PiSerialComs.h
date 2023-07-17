/*
 * PiSerialComs.h
 *
 *  Created on: Sep 12, 2021
 *      Author: Troll
 */

#ifndef ROBOT_PISERIALCOMS_PISERIALCOMS_H_
#define ROBOT_PISERIALCOMS_PISERIALCOMS_H_
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "../Robot/LegControl/LegControl_cfg.h"
#include "../Robot/ADI_IMU/ADI_IMU.h"
#define PSC_BUFFER_SIZE 100
#define PSC_START_CHAR 60
#define PSC_END_CHAR 62
#define PSC_BOARD_ID 1


//the following section defines enums and equivelent strings
#define FOREACH_ACTION(ACTION) \
        ACTION(GETPARAM)   \
        ACTION(SETPARAM)  \
		ACTION(RETPARAM)  \
		ACTION(ACKPARAM)  \
		ACTION(ACTION_ERROR)  \
		ACTION(NUM_ACTIONS) \


#define FOREACH_PARAM(PARAM) \
        PARAM(SPEED)   \
        PARAM(DIST)  \
        PARAM(HEIGHT)   \
        PARAM(DIR)  \
		PARAM(OPLO)  \
		PARAM(STOF)  \
		PARAM(PIDX)  \
		PARAM(PIDY)  \
		PARAM(IMUDATA)  \
		PARAM(PARAM_ERROR)  \
		PARAM(NUM_PARAMS)\

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

enum PARAM_ENUM {
	FOREACH_PARAM(GENERATE_ENUM)
};

static const char *PARAM_STRING[] = {
	FOREACH_PARAM(GENERATE_STRING)
};

enum ACTION_ENUM {
	FOREACH_ACTION(GENERATE_ENUM)
};

static const char *ACTION_STRING[] = {
		FOREACH_ACTION(GENERATE_STRING)
};


typedef struct{
	enum ACTION_ENUM action;
	enum PARAM_ENUM	param;
	float vals[10];
	float numVals;
}PSC_CMD;



typedef struct {                                // object data type
  char Buf[32];
  uint8_t Idx;
} PSC_MSGQUEUE;



extern char PSC_INPUT_BUFFER[PSC_BUFFER_SIZE];
extern volatile char PSC_OUTPUT_BUFFER[PSC_BUFFER_SIZE];
extern char PSC_MESSAGE[PSC_BUFFER_SIZE];
extern volatile int PSC_g_outputDataReady;
extern volatile int PSC_g_outputDataLen;
extern int  PSC_BUFFER_INDEX;
extern int PSC_MSG_LEN;
extern volatile int PSC_NEW_DATA_FROM_BOARD;

void PSC_InitBuffers();

int PSC_checkBuffer();
void PSC_clearBuffer();
int PSC_ProcessCommand(PSC_CMD cmd);
int PSC_InterpretCommand(char msg[], int size);
//void PSC_checkSerial(int fd);
int PSC_SendCmd(PSC_CMD cmd);
int PSC_FindNextToken(char str[], char token[], int start, int len);
int PSC_EvalParam(char str[],unsigned  int len);
int PSC_EvalAction(char str[],unsigned  int len);

int PSC_SendToOutputBuffer(char msg[], int len);
int PSC_ClearOutBuffer();

#endif /* ROBOT_PISERIALCOMS_PISERIALCOMS_H_ */
