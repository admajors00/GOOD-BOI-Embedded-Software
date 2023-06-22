/*
 * SPI_Coms.h
 *
 *  Created on: Feb 10, 2021
 *      Author: Troll
 */

#ifndef SPI_COMS_H_
#define SPI_COMS_H_

#include "stm32f7xx_hal.h"

//const uint8_t READ  = 0x01111111;
#define WRITE  0b11111111

//uint8_t is typedef as unsigned char
//unit16_t is typedef as unsigned short

#define ADI_IMU_READY 1
#define ADI_IMU_NOT_READY 0
//ADIS16507-2 register Definitions
#define DIAG_STAT = 0x02

#define X_GYRO_LOW  0x04
#define X_GYRO_OUT  0x06
#define Y_GYRO_LOW  0x08
#define Y_GYRO_OUT  0x0A
#define Z_GYRO_LOW  0x0C
#define Z_GYRO_OUT  0x0E

#define X_ACCL_LOW  0x10
#define X_ACCL_OUT  0x12
#define Y_ACCL_LOW  0x14
#define Y_ACCL_OUT  0x16
#define Z_ACCL_LOW  0x18
#define Z_ACCL_OUT  0x1A

#define TEMP_OUT  	0x1C
#define DATA_CNTR  	0x22
#define TIME_STAMP 	0x1E

#define GLOB_CMD 	0x68
#define FIRM_REV  	0x6E
#define PROD_ID  	0x72



#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#define WORD_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"
#define WORD_TO_BINARY(word)  \
  (word & 0x8000 ? '1' : '0'), \
  (word & 0x4000 ? '1' : '0'), \
  (word & 0x2000 ? '1' : '0'), \
  (word & 0x1000 ? '1' : '0'), \
  (word & 0x0800 ? '1' : '0'), \
  (word & 0x0400 ? '1' : '0'), \
  (word & 0x0200 ? '1' : '0'), \
  (word & 0x0100 ? '1' : '0'), \
  (word & 0x0080 ? '1' : '0'), \
  (word & 0x0040 ? '1' : '0'), \
  (word & 0x0020 ? '1' : '0'), \
  (word & 0x0010 ? '1' : '0'), \
  (word & 0x0008 ? '1' : '0'), \
  (word & 0x0004 ? '1' : '0'), \
  (word & 0x0002 ? '1' : '0'), \
  (word & 0x0001 ? '1' : '0')

typedef struct{
	uint16_t State;
	SPI_HandleTypeDef HSPI;
	GPIO_TypeDef * CSPort;
	uint16_t CSPin;
	GPIO_TypeDef * DRPort;
	uint16_t DRPin;
	GPIO_TypeDef * RSTPort;
	uint16_t RSTPin;


}ADI_IMU_Device;

extern ADI_IMU_Device ADI_IMU_device1;

extern char uart_buf[1000];
extern int uart_buf_len;
extern uint8_t spi_buf[24];
extern uint8_t ADI_IMU_burstReadBuf[24];
extern float ADI_IMU_burstReadBufScaled[12];
extern float ADI_IMU_gyroAccInitValues[6];
extern uint16_t spi_buf16[12];
extern uint16_t ADI_IMU_accGyroBuf[6];
extern double spi_buf_double[20];
extern uint8_t dataToSend_[2];
extern uint16_t buff;
extern volatile uint8_t spi_xmit_flag;
extern volatile uint8_t spi_recv_flag;


void ADI_IMU_initDevice(SPI_HandleTypeDef hspi, GPIO_TypeDef * csPort, uint16_t csPin, GPIO_TypeDef * drPort, uint16_t drPin, GPIO_TypeDef * rstPort, uint16_t rstPin);
void ADI_IMU_softwareReset();
void ADI_IMU_hardwareReset();

void ADI_IMU_scaleBurstRead();

void ADI_IMU_readRegister(uint8_t thisRegister, uint8_t numBytes, uint8_t *buff);
void ADI_IMU_readRegisterTxRx(uint8_t thisRegister, uint8_t numBytes, uint8_t* buff);
void ADI_IMU_readRegisterTxRx_DMA(uint8_t thisRegister, uint8_t numBytes, uint8_t* buff);


void ADI_IMU_writeRegister( uint8_t thisRegister, uint8_t lowByte, uint8_t highByte);
void ADI_IMU_burstRead();
void ADI_IMU_burstRead_DMA();
void ADI_IMU_readRegisterScaled(uint8_t thisRegister, uint16_t* buff);

void ADI_IMU_checkForErrors();


void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi);
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi);


#endif /* SPI_COMS_H_ */
