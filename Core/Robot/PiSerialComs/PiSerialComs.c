/*
 * PiSerialComs.c
 *
 *  Created on: Sep 12, 2021
 *      Author: Troll
 */
#include "PiSerialComs.h"
char PSC_INPUT_BUFFER[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char PSC_MESSAGE[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int PSC_BUFFER_INDEX = 0;
int PSC_MSG_LEN = 0;

volatile int PSC_NEW_DATA_FROM_BOARD = 0;


int PSC_checkBuffer(){
	for(int i=0; i< PSC_BUFFER_INDEX; i++){
		if(PSC_INPUT_BUFFER[i] == '<' && !PSC_NEW_DATA_FROM_BOARD ){
			for(int j=i+1; j<PSC_BUFFER_INDEX; j++){
				if(PSC_INPUT_BUFFER[j] == '>'&& !PSC_NEW_DATA_FROM_BOARD ){
					for(int k=i+1; k<j; k++){
						PSC_MESSAGE[k-i-1] = PSC_INPUT_BUFFER[k];
						PSC_MSG_LEN = k-i-1;
					}

					//PSC_NEW_DATA_FROM_BOARD = 1;
					PSC_clearBuffer();
					return 1;
					break;
				}
			}
		}

	}
	PSC_MSG_LEN=0;
	return 0;

}

void PSC_clearBuffer(){

	for(int i=0; i<PSC_BUFFER_INDEX; i++){
		PSC_INPUT_BUFFER[i] = '0';
	}
	PSC_BUFFER_INDEX = 0;
}

int PSC_InterpretCommand(char msg[], int size){
	char param[10];
	char value[10];
	char endptr*;
	float floatValue=0;
	int i=0;
	int j=0;
	int k=0;
	for(i=0; i<size; i++){
		if(msg[i] == ','){
			break;		
		}
	}
	if(strncmp(msg, "SETPARAM", i)){
		i++;
		for(j=i; j<size;j++){
			if(msg[j] == ','){
				j++;
				for(k=j; k<size; k++){					
					value[k-j] = msg[k];
				}
				floatValue = strfof(value, &endptr);
				if(*endptr != '\0'){
					return 0;
				}
				break;
			}
			param[j-i]=msg[j];
		}
		if(strncmp(param, "SPEED", j-i)){
			LEG_CONT_g_walkMaxTime = floatValue;
		}
		if(strncmp(param, "DIST", j-i)){
			LEG_CONT_g_walkDistance = floatValue;
		}
		if(strncmp(param, "HEIGHT", j-i)){
			LEG_CONT_g_walkHeight = floatValue;
		}
		if(strncmp(param, "OPLOX", j-i)){
			LEG_CONT_g_walkOpenLoopOffsetX = floatValue;
		}if(strncmp(param, "OPLOY", j-i)){
			LEG_CONT_g_walkOpenLoopOffsetY = floatValue;
		}if(strncmp(param, "DIR", j-i)){
			LEG_CONT_g_walkDirection = floatValue;
		}
	}else{
		return 0;
	}

}

