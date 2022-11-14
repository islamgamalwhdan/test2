/*
 * ili9341.c
 *
 *  Created on: Feb 21, 2021
 *      Author: Islam
 */
#include "ili9341.h"
#include <stm32f4xx_hal.h>

SPI_HandleTypeDef spi;


static void ILI9341_write_data(uint8_t Value);
static void ILI9341_write_command(uint8_t Value);
static void SPI_Init(void);

#define SELECT()      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET)
#define DESELECT()    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET)
#define COMMAND()     HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET)
#define DATA()        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET)


void ILI9341_init(void)
{
	/* initialize control pins */

	  DESELECT();

		/* initialize SPI5 */
	   SPI_Init();

		/* select SPI5 */
	  SELECT();     // only one SPI slave on bus (keep slave selected)
	  ILI9341_write_command(0xCA);
	  	ILI9341_write_data(0xC3);
	  	ILI9341_write_data(0x08);
	  	ILI9341_write_data(0x50);
	  	ILI9341_write_command(ILI9341_POWER_B);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0xC1);
	  	ILI9341_write_data(0x30);
	  	ILI9341_write_command(ILI9341_POWER_SEQUENCE);
	  	ILI9341_write_data(0x64);
	  	ILI9341_write_data(0x03);
	  	ILI9341_write_data(0x12);
	  	ILI9341_write_data(0x81);
	  	ILI9341_write_command(ILI9341_DRIVER_TIMING_CONTROL_A);
	  	ILI9341_write_data(0x85);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x78);
	  	ILI9341_write_command(ILI9341_POWER_A);
	  	ILI9341_write_data(0x39);
	  	ILI9341_write_data(0x2C);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x34);
	  	ILI9341_write_data(0x02);
	  	ILI9341_write_command(ILI9341_PUMP_RATIO_CONTROL);
	  	ILI9341_write_data(0x20);
	  	ILI9341_write_command(ILI9341_DRIVER_TIMING_CONTROL_B);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_command(ILI9341_FRAME_RATE_CONTROL_1);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x1B);
	  	ILI9341_write_command(ILI9341_POWER_CONTROL_1);
	  	ILI9341_write_data(0x10);
	  	ILI9341_write_command(ILI9341_POWER_CONTROL_2);
	  	ILI9341_write_data(0x10);
	  	ILI9341_write_command(ILI9341_VCOM_CONTROL_1);
	  	ILI9341_write_data(0x45);
	  	ILI9341_write_data(0x15);
	  	ILI9341_write_command(ILI9341_VCOM_CONTROL_1);
	  	ILI9341_write_data(0x90);
	  	ILI9341_write_command(ILI9341_MEMORY_ACCESS_CONTROL);
	  	ILI9341_write_data(0xC8);
	  	ILI9341_write_command(ILI9341_3GAMMA_ENABLE);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_command(ILI9341_RGB_INTERFACE_CONTROL);
	  	ILI9341_write_data(0xC2);
	  	ILI9341_write_command(ILI9341_DISPLAY_FUNCTION_CONTROL);
	  	ILI9341_write_data(0x0A);
	  	ILI9341_write_data(0xA7);
	  	ILI9341_write_data(0x27);
	  	ILI9341_write_data(0x04);
	  	ILI9341_write_command(ILI9341_COLUMN_ADDRESS_SET);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0xEF);
	  	ILI9341_write_command(ILI9341_PAGE_ADDRESS_SET);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x01);
	  	ILI9341_write_data(0x3F);
	  	ILI9341_write_command(ILI9341_INTERFACE_CONTROL);
	  	ILI9341_write_data(0x01);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x06);
	  	ILI9341_write_command(ILI9341_GAMMA_REGISTER);
	  	ILI9341_write_data(0x01);
	  	ILI9341_write_command(ILI9341_POSITIVE_GAMMA_CORRECTION);
	  	ILI9341_write_data(0x0F);
	  	ILI9341_write_data(0x29);
	  	ILI9341_write_data(0x24);
	  	ILI9341_write_data(0x0C);
	  	ILI9341_write_data(0x0E);
	  	ILI9341_write_data(0x09);
	  	ILI9341_write_data(0x4E);
	  	ILI9341_write_data(0x78);
	  	ILI9341_write_data(0x3C);
	  	ILI9341_write_data(0x09);
	  	ILI9341_write_data(0x13);
	  	ILI9341_write_data(0x05);
	  	ILI9341_write_data(0x17);
	  	ILI9341_write_data(0x11);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_command(ILI9341_NEGATIVE_GAMMA_CORRECTION);
	  	ILI9341_write_data(0x00);
	  	ILI9341_write_data(0x16);
	  	ILI9341_write_data(0x1B);
	  	ILI9341_write_data(0x04);
	  	ILI9341_write_data(0x11);
	  	ILI9341_write_data(0x07);
	  	ILI9341_write_data(0x31);
	  	ILI9341_write_data(0x33);
	  	ILI9341_write_data(0x42);
	  	ILI9341_write_data(0x05);
	  	ILI9341_write_data(0x0C);
	  	ILI9341_write_data(0x0A);
	  	ILI9341_write_data(0x28);
	  	ILI9341_write_data(0x2F);
	  	ILI9341_write_data(0x0F);
	  	ILI9341_write_command(ILI9341_SLEEP_OUT);
	  	HAL_Delay(200);
	  	ILI9341_write_command(ILI9341_DISPLAY_ON);
}

static void ILI9341_write_command(uint8_t Value)
{
	COMMAND();
	HAL_SPI_Transmit(&spi, (uint8_t*) &Value, 1, HAL_TIMEOUT);
}

static void ILI9341_write_data(uint8_t Value)
{
	DATA();
	HAL_SPI_Transmit(&spi, (uint8_t*) &Value, 1, HAL_TIMEOUT);
}


static void SPI_Init(void)
{

  /* SPI5 parameter configuration*/
	    spi.Instance = SPI5;
		spi.Init.CLKPolarity = SPI_POLARITY_LOW;
		spi.Init.CLKPhase = SPI_PHASE_1EDGE;
		spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
		spi.Init.Mode = SPI_MODE_MASTER;
		spi.Init.DataSize = SPI_DATASIZE_8BIT;
		spi.Init.Direction = SPI_DIRECTION_2LINES;
		spi.Init.NSS = SPI_NSS_SOFT;
		spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
		HAL_SPI_Init(&spi);         // calls HAL_SPI_MspInit()
}
