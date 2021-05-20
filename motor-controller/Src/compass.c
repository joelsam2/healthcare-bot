/*
 * compass.c
 *
 *  Created on: Apr 28, 2021
 *      Author: joels
 */

#include "compass.h"
#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include <stm32f767xx.h>
#include <stm32f7xx_hal_i2c.h>
#include <task.h>
#include <stdio.h>
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart3;

float compass_degree = 0;



void compass_calibrate()
{
	HAL_StatusTypeDef value;

	//value = HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), 0x00, 1, 40);
/*
	int dat[3];
	int command = 0x00;

	uint8_t c = 0x20;
	value = HAL_I2C_Mem_Write(&hi2c2, ((0x60 << 1)), 0x00, I2C_MEMADD_SIZE_8BIT, &c, 1, 40);

	//HAL_I2C_Mem_Read(&hi2c2, (((0x60 << 1)) | 0x01), command, 1, dat, 0, 20);
	HAL_Delay(30);

	c = 0x2A;
	value = HAL_I2C_Mem_Write(&hi2c2, ((0x60 << 1)), 0x00, I2C_MEMADD_SIZE_8BIT, &c, 1, 40);
	I2C_TransferConfig(&hi2c2, ((0x60 << 1)), 1, I2C_AUTOEND_MODE, I2C_GENERATE_STOP);
	//HAL_I2C_Mem_Read(&hi2c2, (((0x60 << 1)) | 0x01), command, 1, dat, 0, 20);
	HAL_Delay(30);

	c = 0x60;
	HAL_I2C_Mem_Write(&hi2c2, ((0x60 << 1)), 0x00, I2C_MEMADD_SIZE_8BIT, &c, 1, 40);
	I2C_TransferConfig(&hi2c2, ((0x60 << 1)), 1, I2C_AUTOEND_MODE, I2C_GENERATE_STOP);
	//HAL_I2C_Mem_Read(&hi2c2, (((0x60 << 1)) | 0x01), command, 1, dat, 0, 20);
	HAL_Delay(4000);

	c = 0xF8;
	HAL_I2C_Mem_Write(&hi2c2, ((0x60 << 1)), 0x00, I2C_MEMADD_SIZE_8BIT, &c, 1, 40);
	I2C_TransferConfig(&hi2c2, ((0x60 << 1)), 1, I2C_AUTOEND_MODE, I2C_GENERATE_STOP);
	//HAL_I2C_Mem_Read(&hi2c2, (((0x60 << 1)) | 0x01), command, 1, dat, 0, 20);
	HAL_Delay(30);

*/
	uint8_t outbuffer[2] = {0x00, 0x20};
	HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), &outbuffer[0], 1, 40);
	I2C_TransferConfig(&hi2c2, ((0x60 << 1)), 1, I2C_AUTOEND_MODE, I2C_GENERATE_STOP);
	HAL_Delay(25);

	outbuffer[0] = 0x00;
	outbuffer[1] = 0xF0;
	HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), &outbuffer[0], 1, 40);
	I2C_TransferConfig(&hi2c2, ((0x60 << 1)), 1, I2C_AUTOEND_MODE, I2C_GENERATE_STOP);
	HAL_Delay(25);

	outbuffer[0] = 0x00;
	outbuffer[1] = 0xF5;
	HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), &outbuffer[0], 1, 40);
	I2C_TransferConfig(&hi2c2, ((0x60 << 1)), 1, I2C_AUTOEND_MODE, I2C_GENERATE_STOP);
	HAL_Delay(2000);

	outbuffer[0] = 0x00;
	outbuffer[1] = 0xF6;
	HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), &outbuffer[0], 1, 40);
	I2C_TransferConfig(&hi2c2, ((0x60 << 1)), 1, I2C_AUTOEND_MODE, I2C_GENERATE_STOP);
	HAL_Delay(25);


	uint8_t outbuffer[2] = {0x00, 0x20};
	HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), &outbuffer[0], 2, 40);
	HAL_Delay(30);

	outbuffer[0] = 0x00;
	outbuffer[1] = 0x2A;
	HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), &outbuffer[0], 2, 40);
	HAL_Delay(30);

	outbuffer[0] = 0x00;
	outbuffer[1] = 0x60;
	HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), &outbuffer[0], 2, 40);
	HAL_Delay(3000);

	outbuffer[0] = 0x00;
	outbuffer[1] = 0xF8;
	HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), &outbuffer[0], 2, 40);
	HAL_Delay(30);
}

void get_compass_value()
{
	  uint8_t dat[2] = {0};
	  uint8_t command = 0x02;
	  char temp[40];
	  static int count = 0;

	  HAL_StatusTypeDef value_compass;

	  //taskENTER_CRITICAL();
	  value_compass = HAL_I2C_Master_Transmit(&hi2c2, (0x60 << 1), &command, 1, 100);

	  if (value_compass == HAL_OK)
	  {
		  value_compass = HAL_I2C_Mem_Read(&hi2c2, ((0x60 << 1) | 0x01), command, 1, dat, 2, 100);

		if (value_compass == HAL_OK)
		{
		  uint16_t data = dat[0] << 8 | dat[1];
		  compass_degree = data * 0.1f; // convert raw data to degree value.

		  count++;

		  if(count > 200){
			  sprintf(temp, "%d\r\n",(data));
			  HAL_UART_Transmit(&huart3, temp, strlen(temp), 50);
			  count = 0;
		  }
		}
	  }

	  //taskEXIT_CRITICAL();
}
