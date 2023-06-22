/*
 * SPI_Coms.c
 *
 *  Created on: Feb 10, 2021
 *      Author: Troll
 */

#include "ADI_IMU.h"


uint8_t burstReadRegisters[9] ={0x02, 0x06, 0x0A, 0x0E, 0x12, 0x16, 0x1A, 0x1C, 0x22};
uint8_t burstReadRegisters2[24] ={0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


char uart_buf[1000] ={0};
 int uart_buf_len =0;
 uint8_t spi_buf[24] ={};
 uint8_t ADI_IMU_burstReadBuf[24] ={0};
 float ADI_IMU_burstReadBufScaled[12] ={0};
 float ADI_IMU_gyroAccInitValues[6] ={0};
 uint16_t spi_buf16[12] ={0};
 uint16_t ADI_IMU_accGyroBuf[6] ={0};
 double spi_buf_double[20] ={0};
 uint8_t dataToSend_[2] ={0};
 uint16_t buff =0;
 volatile uint8_t spi_xmit_flag =0;
 volatile uint8_t spi_recv_flag =0;
//volatile is required for changing variables in main and ISR

 ADI_IMU_Device ADI_IMU_device1;
//static uint8_t rxBuf[20];
//static uint8_t txBuf[20];


void ADI_IMU_initDevice(SPI_HandleTypeDef hspi, GPIO_TypeDef * csPort, uint16_t csPin, GPIO_TypeDef * drPort, uint16_t drPin, GPIO_TypeDef * rstPort, uint16_t rstPin){
	ADI_IMU_device1.State = ADI_IMU_NOT_READY;

	ADI_IMU_device1.HSPI = hspi;
	ADI_IMU_device1.CSPort = csPort;
	ADI_IMU_device1.CSPin = csPin;
	ADI_IMU_device1.DRPort = drPort;
	ADI_IMU_device1.DRPin = drPin;
	ADI_IMU_device1.RSTPort = rstPort;
	ADI_IMU_device1.RSTPin = rstPin;

	//HAL_GPIO_WritePin(rstPort, rstPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
	//HAL_Delay(1);
	ADI_IMU_hardwareReset();

	ADI_IMU_device1.State = ADI_IMU_READY;



}

void ADI_IMU_softwareReset(){
	//HAL_GPIO_WritePin(ADI_IMU_device1.RSTPort, ADI_IMU_device1.RSTPin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(ADI_IMU_device1.RSTPort, ADI_IMU_device1.RSTPin, GPIO_PIN_SET);
	//reset recovery time ~255ms
	ADI_IMU_device1.State = ADI_IMU_NOT_READY;
	uint8_t lowByteToSend[2];
	  lowByteToSend[0]= 0xE8;
	  lowByteToSend[1] = 0x80;

	  uint8_t highByteToSend[2];
	  highByteToSend[0]= 0xE9;
	  highByteToSend[1] = 0x00;

	  //send the value
	  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&ADI_IMU_device1.HSPI , (uint8_t *)&highByteToSend, 2, 100);
	  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);
	  //send the address
	  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&ADI_IMU_device1.HSPI , (uint8_t *)&lowByteToSend, 2, 100);
	  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);
	 // HAL_Delay(255);
	  ADI_IMU_device1.State = ADI_IMU_READY;

}
void ADI_IMU_hardwareReset(){
	ADI_IMU_device1.State = ADI_IMU_NOT_READY;
	HAL_GPIO_TogglePin(ADI_IMU_device1.RSTPort, ADI_IMU_device1.RSTPin);
	HAL_GPIO_TogglePin(ADI_IMU_device1.RSTPort, ADI_IMU_device1.RSTPin);
	//HAL_Delay(255);
	ADI_IMU_device1.State = ADI_IMU_READY;

}

void ADI_IMU_scaleBurstRead(){
	int j;
 /* empty, empty, DIAG_STAT, X_GYRO_OUT, Y_GYRO_OUT, Z_GYRO_OUT, X_ACCL_OUT, Y_ACCL_OUT, Z_ACCL_OUT, TEMP_OUT, DATA_CNTR, check sum*/
/*   0,1	2,3      4,5        6,7         8,9         10,11      12,13        14,15       16,17      18,19     20,21      22, 23*/
	for(int i=0; i<24; i+=2){
		j = i/2;
		ADI_IMU_burstReadBufScaled[j] = ((((int16_t)ADI_IMU_burstReadBuf[i]<<8) & 0xFF00) | (((int16_t)ADI_IMU_burstReadBuf[i+1]) & 0x00FF));
		if((int16_t)ADI_IMU_burstReadBufScaled[j] & 0x8000){
			ADI_IMU_burstReadBufScaled[j] = -1*(~(int16_t)ADI_IMU_burstReadBufScaled[j]+1);
		}
		if(j >= 2 && j <=4){
			ADI_IMU_burstReadBufScaled[j] *= 500.0/20000.0;
		}
		if(j >= 5 && j <= 7){
			ADI_IMU_burstReadBufScaled[j] *= 392.0/32000.0;
		}
		if(j == 8){
			ADI_IMU_burstReadBufScaled[j] = (ADI_IMU_burstReadBufScaled[j]*105.0/1050.0 * 9.0/5.0) + 32;
		}

	}
}

void ADI_IMU_readRegister(uint8_t thisRegister, uint8_t numBytes, uint8_t* buff){
  uint8_t dataToSend[2];
  dataToSend[1]= thisRegister;
  dataToSend[0] = 0x00;


  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&ADI_IMU_device1.HSPI, (uint8_t*)&dataToSend,  2, 100);
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);

 // HAL_Delay(1);

  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
  HAL_SPI_Receive(&ADI_IMU_device1.HSPI , (uint8_t*)&spi_buf,  2, 100);
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);
}

void ADI_IMU_readRegisterTxRx(uint8_t thisRegister, uint8_t numBytes, uint8_t* buff){
  uint8_t dataToSend[2];
  dataToSend[1]= thisRegister;
  dataToSend[0] = 0x00;


  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&ADI_IMU_device1.HSPI, (uint8_t*)&dataToSend, (uint8_t*)&spi_buf, 1, 100);
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);

}

void ADI_IMU_readRegisterTxRx_DMA(uint8_t thisRegister, uint8_t numBytes, uint8_t* buff){
  uint8_t dataToSend[2];
  dataToSend[1]= thisRegister;
  dataToSend[0] = 0x00;

  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_DMA(&ADI_IMU_device1.HSPI, (uint8_t*)&dataToSend,(uint8_t*)&spi_buf,   1);
  //HAL_SPI_Receive_DMA(&ADI_IMU_device1.HSPI, (uint8_t*)&buff,  2);

}


void ADI_IMU_writeRegister(uint8_t thisRegister, uint8_t lowByte, uint8_t highByte){
  uint8_t lowByteToSend[2];
  lowByteToSend[1]= thisRegister;
  lowByteToSend[0] = lowByte;

  uint8_t highByteToSend[2];
  highByteToSend[1]= thisRegister + 1;
  highByteToSend[0] = highByte;


  //send the address
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&ADI_IMU_device1.HSPI , (uint8_t *)&lowByteToSend, 2, 100);
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);

  //send the value
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&ADI_IMU_device1.HSPI , (uint8_t *)&highByteToSend, 2, 100);
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);
}

void ADI_IMU_burstRead(){

  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&ADI_IMU_device1.HSPI , (uint8_t*)&burstReadRegisters2, (uint8_t*)&ADI_IMU_burstReadBuf,  24, 100);

  //HAL_SPI_Receive(&hspi1, (uint8_t*)spi_buf,  22, 100);
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);


}
void ADI_IMU_burstRead_DMA(){
	ADI_IMU_device1.State = ADI_IMU_NOT_READY;
	HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive_DMA(&ADI_IMU_device1.HSPI , (uint8_t*)&burstReadRegisters2, (uint8_t*)&ADI_IMU_burstReadBuf,  24) != HAL_OK){
		while(1){}
	}
 // HAL_SPI_Transmit_DMA(&ADI_IMU_device1.HSPI , (uint8_t*)&burstReadRegisters2,  24);


  //HAL_SPI_Receive(&hspi1, (uint8_t*)spi_buf,  22, 100);
}

void ADI_IMU_readRegisterScaled(uint8_t thisRegister, uint16_t* buff){
  uint8_t regVals[2];
  uint16_t regVal = 0;
  uint8_t dataToSend[2];
  dataToSend[0]= thisRegister;
  dataToSend[1] = 0x00;

  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&ADI_IMU_device1.HSPI , (uint8_t*)&dataToSend,  2, 100);
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);

  HAL_Delay(1);
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
  HAL_SPI_Receive(&ADI_IMU_device1.HSPI , (uint8_t*)regVals,  2, 100);
  HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);

  //&with 0xff00 bc garbage can be shifted in
  regVal = ((regVals[0]<<8) & 0xFF00) | regVals[1];
  spi_buf16[0] = regVal;

}

//void ADI_IMU_checkForErrors(){
//  ADI_IMU_readRegister(DIAG_STAT, 2, spi_buf);
//  int flag = 0;
//  int var = (int)spi_buf[0];
//  switch(var){
//	  case 0b0000000000000001:
//		  flag = 1;
//		  break;
//	  case 0b0000000000000010:
//		  uart_buf_len = sprintf(uart_buf, "Data PAth OverRun\r\n");
//		flag = 1;
//		  break;
//	  case 0b000000000000100:
//		  uart_buf_len = sprintf(uart_buf, "Flash Memory Update Failure\r\n");
//		flag = 1;
//		  break;
//	  case 0b0000000000001000:
//		  uart_buf_len = sprintf(uart_buf, "SPI Comms Error\r\n");
//		flag = 1;
//		  break;
//	  case 0b0000000000010000:
//		  uart_buf_len = sprintf(uart_buf, "StandBy Mode\r\n");
//		flag = 1;
//		  break;
//	  case 0b0000000000100000:
//		  uart_buf_len = sprintf(uart_buf, "Sensor Failure\r\n");
//		flag = 1;
//		  break;
//	  case 0b0000000001000000:
//		  uart_buf_len = sprintf(uart_buf, "Memory Failure\r\n");
//		flag = 1;
//		  break;
//	  case 0b0000000010000000:
//		  uart_buf_len = sprintf(uart_buf, "Clock Error\r\n");
//		flag = 1;
//		  break;
//  }
//
//  if(flag){
//	  HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, uart_buf_len, 100);
//	  HAL_Delay(500);
//		  while(1){
//			  //stay here
//		  }
//  }
//}



//called when spi transmit is done
void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi){
	//HAL_SPI_Receive_DMA(&ADI_IMU_device1.HSPI , (uint8_t*)spi_buf,  24);
	HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&ADI_IMU_device1.HSPI , (uint8_t*)spi_buf,  1, 100);
	HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);
	spi_xmit_flag = 0;

}

//called when spi recieve is done
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi){
	HAL_GPIO_WritePin(ADI_IMU_device1.CSPort, ADI_IMU_device1.CSPin, GPIO_PIN_SET);
	spi_recv_flag = 0;

}
