/*
 * PiSerialComs.h
 *
 *  Created on: Sep 12, 2021
 *      Author: Troll
 */

#ifndef ROBOT_PISERIALCOMS_PISERIALCOMS_H_
#define ROBOT_PISERIALCOMS_PISERIALCOMS_H_
#include <stdio.h>
#define PSC_BUFFER_SIZE 32
#define PSC_START_CHAR 60
#define PSC_END_CHAR 62
#define PSC_BOARD_ID 1

typedef struct {                                // object data type
  char Buf[32];
  uint8_t Idx;
} PSC_MSGQUEUE;

extern char PSC_INPUT_BUFFER[PSC_BUFFER_SIZE];
extern char PSC_MESSAGE[PSC_BUFFER_SIZE];

extern int  PSC_BUFFER_INDEX;
extern int PSC_MSG_LEN;
extern volatile int PSC_NEW_DATA_FROM_BOARD;

int PSC_checkBuffer();
void PSC_clearBuffer();
//void PSC_checkSerial(int fd);

#endif /* ROBOT_PISERIALCOMS_PISERIALCOMS_H_ */
