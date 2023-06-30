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
#include <stdio.h>
#include <stdlib.h>
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

int PSC_InterpretCommand(char msg[], int size) {
	char param[10];
	char value[10];
	char *endptr;
	float floatValue = 0;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int k = 0;
	int msgFound = 0;
	unsigned int paramLen = 0;
	for (i = 0; i < size; i++) {
		if (msg[i] == ',') {
			msgFound = 1;
			break;
		}
	}
	if (msgFound) {
		if (strncmp(msg, "SETPARAM", i - 1) == 0) {
			i++;
			for (j = i; j < size; j++) {
				if (msg[j] == ',') {
					j++;
					for (k = j; k < size; k++) {
						value[k - j] = msg[k];
					}
					floatValue = strtof(value, &endptr);
					if (*endptr != '\0') {
						return 0;
					}
					break;
				}
				floatValue = strtof(value, &endptr);
				if (*endptr != '\0') {
					return 0;
				}
				break;
				param[j - i] = msg[j];
			}

			paramLen = j - i - 1;
			if (strncmp(param, "SPEED", paramLen) == 0 && paramLen == 5) {
				LEG_CONT_g_walkMaxTime = floatValue;
				return 1;
			} else if (strncmp(param, "DIST", paramLen) == 0 && paramLen == 4) {
				LEG_CONT_g_walkDistance = floatValue;
				return 1;
			} else if (strncmp(param, "HEIGHT", paramLen) == 0
					&& paramLen == 5) {
				LEG_CONT_g_walkHeight = floatValue;
				return 1;
			} else if (strncmp(param, "OPLOX", paramLen) == 0
					&& paramLen == 5) {
				LEG_CONT_g_walkOpenLoopOffsetX = floatValue;
				return 1;
			} else if (strncmp(param, "OPLOY", paramLen) == 0
					&& paramLen == 5) {
				LEG_CONT_g_walkOpenLoopOffsetY = floatValue;
				return 1;
			} else if (strncmp(param, "DIR", j - i)) {
				LEG_CONT_g_walkDirection = floatValue;
				return 1;
			} else {
				return 0;
			}
		} else {
			return 0;
		}

	}
	return 0;
}


