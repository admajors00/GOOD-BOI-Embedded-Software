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
	//std::cout<<"Check Buffer \n";
	//std::cout << "\tINPUT_BUFFER: " << INPUT_BUFFER[BUFF_INDEX-1] << std::endl;
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
						return 1;//PSC_clearBuffer();
						break;
					}
				}
			}

		}
		PSC_MSG_LEN=0;
		return 0;
	//std::cout << "MSG: " << MSG << std::endl;
}

void PSC_clearBuffer(){
 // Serial.println(inputBuffer);
  //for(int i=0;i<BUFF_SIZE; i++){
  //  INPUT_BUFFER[i] = '\0';
  //}
	for(int i=0; i<PSC_BUFFER_INDEX; i++){
		PSC_INPUT_BUFFER[i] = '0';
	}
	PSC_BUFFER_INDEX = 0;
}

//void PSC_checkSerial(int fd){
//  while (serialDataAvail(fd) > 0){
//	 // PSC_INPUT_BUFFER += serialGetchar(fd);
//	  PSC_BUFFER_INDEX++;
//  }

  //std::cout << "INPUT_BUFFER: " << INPUT_BUFFER << std::endl;
  //std::cout << "BUFFER INDEX: " << BUFF_INDEX << std::endl;

//}
