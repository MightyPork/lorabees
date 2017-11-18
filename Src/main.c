/*
/ _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
\____ \| ___ |    (_   _) ___ |/ ___)  _ \
_____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
   (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    08-September-2017
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <hw_i2c.h>
#include <stm32l0xx_ll_i2c.h>
#include "stm32l0xx_ll_i2c.h"
#include "hw.h"
#include "low_power.h"
#include "lora.h"
//#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "voc_sensor.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LPP_APP_PORT 99

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            10000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1
/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG                    DISABLE
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            42
//2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            3

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa will transmit a frame*/
static void LoraTxData(lora_AppData_t *AppData, FunctionalState *IsTxConfirmed);

/* call back when LoRa has received a frame*/
static void LoraRxData(lora_AppData_t *AppData);

uint8_t HW_GetBatteryLevel(void) {
	return 254;
}

/* Private variables ---------------------------------------------------------*/
/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks = {HW_GetBatteryLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LoraTxData,
                                               LoraRxData};


#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent( void );
#endif
/* !
 *Initialises the Lora Parameters
 */
static LoRaParam_t LoRaParamInit = {TX_ON_TIMER,
                                    APP_TX_DUTYCYCLE,
                                    CLASS_A,
                                    LORAWAN_ADR_ON,
                                    DR_0,
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

/* Private functions ---------------------------------------------------------*/

#if 0
typedef enum {
	i2cSpeed_std,
	i2cSpeed_fast,
	i2cSpeed_fastPlus,
	i2cSpeed_count,
} i2cSpeed_t;

void i2cInit(I2C_TypeDef *i2c, i2cSpeed_t spd);

#define I2C_7BIT_ADDR (0 << 31)
#define I2C_10BIT_ADDR (1 << 31)

// Returns number of bytes written
uint32_t i2cWrite(I2C_TypeDef *i2c, uint32_t addr, uint8_t *txBuffer,
				  uint32_t len);

// Returns number of bytes read
uint32_t i2cRead(I2C_TypeDef *i2c, uint8_t addr, uint8_t *rxBuffer,
				 uint32_t numBytes);

#define I2C_READ 0
#define I2C_WRITE 1

static uint32_t setupTiming(i2cSpeed_t spd, uint32_t clockFreq) {
	(void) spd;
	(void) clockFreq;
	uint32_t presc = 0;
	uint32_t sdadel = 2;
	uint32_t scldel = 2;
	uint32_t scll = 6;
	uint32_t sclh = 7;

	return  presc << 28 |
			scldel << 20 |
			sdadel << 16 |
			sclh << 8 |
			scll;
}

void i2cInit(I2C_TypeDef *i2c, i2cSpeed_t spd) {
	// Setup timing register
	i2c->TIMINGR = setupTiming(spd, SystemCoreClock);

	// Reset state
	i2c->CR1 &= ~I2C_CR1_PE;
}

static uint32_t i2cSetup(uint32_t addr, uint8_t direction) {
	uint32_t ret = 0;
	if (addr & I2C_10BIT_ADDR) {
		ret = (addr & 0x000003FF) | I2C_CR2_ADD10;
	} else {
		// 7 Bit Address
		ret = (addr & 0x0000007F) << 1;
	}

	if (direction == I2C_READ) {
		ret |= I2C_CR2_RD_WRN;
		if (addr & I2C_10BIT_ADDR) {
			ret |= I2C_CR2_HEAD10R;
		}
	}

	return ret;
}

// Will return the number of data bytes written to the device
uint32_t i2cWrite(I2C_TypeDef *i2c, uint32_t addr, uint8_t *txBuffer,
				  uint32_t len) {

	uint32_t numTxBytes = 0;

	i2c->CR1 &= ~I2C_CR1_PE;
	i2c->CR2 = 0;

	i2c->CR2 = i2cSetup(addr, I2C_WRITE);

	if (len > 0xFF) {
		i2c->CR2 |= 0x00FF0000 | I2C_CR2_RELOAD;
	} else {
		i2c->CR2 |= ((len & 0xFF) << 16) | I2C_CR2_AUTOEND;
	}
	i2c->CR1 |= I2C_CR1_PE;
	i2c->CR2 |= I2C_CR2_START;
	while(i2c->CR2 & I2C_CR2_START);
	uint8_t done = 0;
	uint32_t i = 0;
	while (!done && i < 0x0000001F) {
		i++;
		if (i2c->ISR & I2C_ISR_NACKF) {
			// Was not acknowledged, disable device and exit
			done = 1;
		}

		if (i2c->ISR & I2C_ISR_TXIS) {
			// Device acknowledged and we must send the next byte
			if (numTxBytes < len){
				i2c->TXDR = txBuffer[numTxBytes++];
			}

			i = 0;

		}

		if (i2c->ISR & I2C_ISR_TC) {
			done = 1;
		}

		if (i2c->ISR & I2C_ISR_TCR) {
			i = 0;
			if ((len - numTxBytes) > 0xFF) {
				i2c->CR2 |= 0x00FF0000 | I2C_CR2_RELOAD;
			} else {
				i2c->CR2 &= ~(0x00FF0000 | I2C_CR2_RELOAD);
				i2c->CR2 |= ((len - numTxBytes) & 0xFF) << 16 |
							I2C_CR2_AUTOEND;
			}
		}

	}
	i2c->CR1 &= ~I2C_CR1_PE;
	return numTxBytes;
}

uint32_t i2cRead(I2C_TypeDef *i2c, uint8_t addr, uint8_t *rxBuffer,
				 uint32_t numBytes) {

	uint32_t numRxBytes = 0;

	i2c->CR1 &= ~I2C_CR1_PE;
	i2c->CR2 = 0;

	i2c->CR2 = i2cSetup(addr, I2C_READ);

	if (numBytes > 0xFF) {
		i2c->CR2 |= 0x00FF0000 | I2C_CR2_RELOAD;
	} else {
		i2c->CR2 |= ((numBytes & 0xFF) << 16) | I2C_CR2_AUTOEND;
	}
	i2c->CR1 |= I2C_CR1_PE;
	i2c->CR2 |= I2C_CR2_START;

	while(i2c->CR2 & I2C_CR2_START);
	uint8_t done = 0;
	uint32_t i = 0;
	while (!done && i < 0x0000001F) {
		i++;

		if (i2c->ISR & I2C_ISR_RXNE) {
			// Device acknowledged and we must send the next byte
			if (numRxBytes < numBytes){
				rxBuffer[numRxBytes++] = i2c->RXDR;
			}

			i = 0;
		}

		if (i2c->ISR & I2C_ISR_TC) {
			done = 1;
		}

		if (i2c->ISR & I2C_ISR_TCR) {
			i = 0;
			if ((numBytes - numRxBytes) > 0xFF) {
				i2c->CR2 |= 0x00FF0000 | I2C_CR2_RELOAD;
			} else {
				i2c->CR2 &= ~(0x00FF0000 | I2C_CR2_RELOAD);
				i2c->CR2 |= ((numBytes - numRxBytes) & 0xFF) << 16 |
							I2C_CR2_AUTOEND;
			}
		}

	}
	i2c->CR1 &= ~I2C_CR1_PE;
	return numRxBytes;
}
#endif


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/* STM32 HAL library initialization*/
	HAL_Init();

	/* Configure the system clock*/
	SystemClock_Config();

	/* Configure the debug mode*/
	DBG_Init();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Configure the hardware*/
	HW_Init();
	MX_I2C1_Init();

	// BLINKY
	GPIO_InitTypeDef initStruct = { 0 };
	initStruct.Mode =GPIO_MODE_OUTPUT_PP;
	initStruct.Pull = GPIO_NOPULL;
	initStruct.Speed = GPIO_SPEED_HIGH;
	HW_GPIO_Init(GPIOC, GPIO_PIN_7, &initStruct);

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */
	voc_init();

	while(1) {
		GPIOC->ODR ^= 1<<7;
		voc_measure();
	}
#if 0

	/* Configure the Lora Stack*/
	lora_Init(&LoRaMainCallbacks, &LoRaParamInit);

	PRINTF("starting!!!\n\r");

	/* main loop*/
	while (1) {

		/* run the LoRa class A state machine*/
		lora_fsm();

		DISABLE_IRQ();
		/* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
		 * and cortex will not enter low power anyway  */
		if (lora_getDeviceState() == DEVICE_STATE_SLEEP) {
#ifndef LOW_POWER_DISABLE
			LowPower_Handler();
#endif
		}
		ENABLE_IRQ();

		/* USER CODE BEGIN 2 */
		/* USER CODE END 2 */
	}
#endif
}

static void LoraTxData(lora_AppData_t *AppData, FunctionalState *IsTxConfirmed)
{
	static uint8_t counter = 0;
	/* USER CODE BEGIN 3 */
	PRINTF("Lora TX\r\n");

	AppData->Port = LORAWAN_APP_PORT;
	*IsTxConfirmed = LORAWAN_CONFIRMED_MSG;

	sprintf((char*)AppData->Buff, "HELLO-%02d", counter++);
	AppData->BuffSize = 8;

	/* USER CODE END 3 */
}

static void LoraRxData(lora_AppData_t *AppData)
{
	/* USER CODE BEGIN 4 */
	PRINTF("Lora RX\r\n");

	switch (AppData->Port) {
		case LORAWAN_APP_PORT:

			break;
		case LPP_APP_PORT: {

			break;
		}
		default:
			break;
	}
	/* USER CODE END 4 */
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
