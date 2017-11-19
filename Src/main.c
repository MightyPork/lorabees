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
#include "stm32l0xx_ll_i2c.h"
#include "hw.h"
#include "low_power.h"
#include "lora.h"
//#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "voc_sensor.h"
#include "payload_builder.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LPP_APP_PORT 99

/*!
 * Defines the application data transmission duty cycle. value in [ms].
 */
#if 0
#define APP_TX_DUTYCYCLE                            (2*60*1000)
#define MEAS_TX_DELAY 								(15*1000) // offset of the first measurement (negative) to throw it off from sync with TX
#define MEAS_INTERVAL_MS							(1*60*1000)
#else
#define APP_TX_DUTYCYCLE                            (15*60*1000)
#define MEAS_TX_DELAY 								(   15*1000) // offset of the first measurement (negative) to throw it off from sync with TX
#define MEAS_INTERVAL_MS							( 5*60*1000)
#endif
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
#define LORAWAN_APP_PORT                            68
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
									(bool)LORAWAN_ADR_ON,
									DR_0,
									(bool)LORAWAN_PUBLIC_NETWORK,
									JOINREQ_NBTRIALS};

/* Private functions ---------------------------------------------------------*/

static TimerEvent_t MeasurementStartTimer;
static struct bme680_field_data voc_data;

void MeasurementStartTimerIrq(void)
{
	GPIOC->ODR |= 1<<7;
	PRINTF("--- MEASUREMENT CALLBACK! --- \r\n");

	TimerSetValue( &MeasurementStartTimer, MEAS_INTERVAL_MS );
	TimerReset(&MeasurementStartTimer);

	uint32_t duration = voc_start_measure();
	HAL_Delay(duration); // this is usually like 200 ms, not enough to worry about sleep
	voc_read(&voc_data);

	// STUFF...
	GPIOC->ODR &= ~(1<<7);
}


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

	// BLINKY
	GPIO_InitTypeDef initStruct = { 0 };
	initStruct.Mode =GPIO_MODE_OUTPUT_PP;
	initStruct.Pull = GPIO_NOPULL;
	initStruct.Speed = GPIO_SPEED_HIGH;
	HW_GPIO_Init(GPIOC, GPIO_PIN_7, &initStruct);

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */
	voc_init();

	/* Configure the Lora Stack*/
	lora_Init(&LoRaMainCallbacks, &LoRaParamInit);

    TimerInit( &MeasurementStartTimer, MeasurementStartTimerIrq );
	TimerSetValue( &MeasurementStartTimer, MEAS_INTERVAL_MS - MEAS_TX_DELAY ); // first time with a delay, to get some offset
	TimerStart( &MeasurementStartTimer );

	PRINTF("Initial sensor measurement...\r\n");
	MeasurementStartTimerIrq();

	/* main loop*/
	PRINTF("Main loop starting!!!\n\r");
	while (1) {
		GPIOC->ODR ^= 1<<7;

		/* run the LoRa class A state machine*/
		lora_fsm();

		DISABLE_IRQ();
		/* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
		 * and cortex will not enter low power anyway  */
		if (lora_getDeviceState() == DEVICE_STATE_SLEEP) {
#ifndef LOW_POWER_DISABLE

			GPIOC->ODR &= ~(1<<7); // LED off
			LowPower_Handler();
#endif
		}
		ENABLE_IRQ();

		/* USER CODE BEGIN 2 */
		/* USER CODE END 2 */
	}
}

static void LoraTxData(lora_AppData_t *AppData, FunctionalState *IsTxConfirmed)
{
	/* USER CODE BEGIN 3 */
	PRINTF("Lora TX\r\n");

	AppData->Port = LORAWAN_APP_PORT;
	*IsTxConfirmed = LORAWAN_CONFIRMED_MSG;

	PayloadBuilder pb = pb_start(AppData->Buff, AppData->BuffSize, true);
	pb_i16(&pb, voc_data.temperature); // Cx100
	pb_u16(&pb, (uint16_t) (voc_data.humidity / 10)); // discard one place -> %x100
	pb_u16(&pb, (uint16_t) (voc_data.pressure - 85000)); // send offset from 850 hPa -> Pa
	pb_u32(&pb, (uint16_t) (voc_data.gas_resistance)); // ohms, full size
	AppData->BuffSize = (uint8_t) pb_length(&pb);

	/* USER CODE END 3 */
}

static void LoraRxData(lora_AppData_t *AppData)
{
	PRINTF("Lora RX\r\n");
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
