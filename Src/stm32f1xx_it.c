/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#include "led_segment.h"
#include "stm32f1xx_hal_flash.h"

uint8_t dc = 0;
uint8_t pc = 0;
uint8_t pt = 0;

uint32_t data = 0;

extern volatile uint16_t i;
extern volatile uint8_t p;
extern volatile uint8_t t;
extern volatile uint8_t h;
extern volatile uint8_t mode;

extern volatile uint8_t buf1, buf2, buf3;

extern uint32_t Address;
extern uint32_t PAGEError;

extern double value;

extern ADC_HandleTypeDef hadc1;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	NVIC_DisableIRQ(EXTI0_IRQn);
	__HAL_TIM_CLEAR_IT(&htim16, TIM_IT_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim16, 0);
	HAL_TIM_Base_Start_IT(&htim16);
	mode++;
	if(mode > 3) mode = 0;
	NVIC_DisableIRQ(TIM6_DAC_IRQn);
	if(mode == 0) {
		HAL_TIM_Base_Stop(&htim16);
		buf1 = '-';
		buf2 = '-';
		buf3 = '-';
		data = p;
		data <<= 8;
		data |= t;
		data <<= 8;
		data |= h;
		HAL_FLASH_Unlock();
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
		EraseInitStruct.NbPages = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
		HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
		Address = FLASH_USER_START_ADDR;
		while(Address < FLASH_USER_END_ADDR) {
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data) == HAL_OK) {
				Address = Address + 4;
			}
		}
		HAL_FLASH_Lock();
	}
	if(mode == 1) {
		buf1 = 't';
		buf2 = '-';
		buf3 = ' ';
	}
	if(mode == 2) {
		buf1 = 'P';
		buf2 = '-';
		buf3 = ' ';
	}
	if(mode == 3) {
		buf1 = 'h';
		buf2 = '-';
		buf3 = ' ';
	}
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	HAL_Delay(1000);
	NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	NVIC_DisableIRQ(EXTI1_IRQn);
	__HAL_TIM_SET_COUNTER(&htim16, 0);
	if(mode == 1) {
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		t++;
		if(t > 250) t = 0;
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_Delay(300);
	}
	if(mode == 2) {
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		p++;
		if(p > 100) p = 0;
		pt = p;
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_Delay(300);
	}
	if(mode == 3) {
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		h++;
		if(h > 20) h = 0;
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_Delay(300);
	}
	NVIC_EnableIRQ(EXTI1_IRQn);
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	NVIC_DisableIRQ(EXTI2_IRQn);
	__HAL_TIM_SET_COUNTER(&htim16, 0);
	if(mode == 1) {
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		if(t == 0) t = 251;
		t--;
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_Delay(300);
	}
	if(mode == 2) {
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		if(p == 0) p = 101;
		p--;
		pt = p;
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_Delay(300);
	}
	if(mode == 3) {
		NVIC_DisableIRQ(TIM6_DAC_IRQn);
		if(h == 0) h = 21;
		h--;
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_Delay(300);
	}
	NVIC_EnableIRQ(EXTI2_IRQn);
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
*/
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
	HAL_TIM_Base_Stop(&htim16);
	NVIC_DisableIRQ(TIM6_DAC_IRQn);
	mode = 0;
	buf1 = '-';
	buf2 = '-';
	buf3 = '-';
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	HAL_Delay(500);
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
* @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
*/
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	if(HAL_ADC_GetValue(&hadc1) < 0x000A) pc = 0;
	if(pc > 99) HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, GPIO_PIN_RESET);
	else if(pc == (100 - pt)) HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, GPIO_PIN_SET);
	HAL_ADC_Stop(&hadc1);
	pc++;
  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt and DAC underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	HAL_GPIO_WritePin(DIGIT_1_GPIO_Port, DIGIT_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DIGIT_2_GPIO_Port, DIGIT_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DIGIT_3_GPIO_Port, DIGIT_3_Pin, GPIO_PIN_RESET);
	Display_Clear();
	dc++;
	if(dc > 2) dc = 0;
	if(dc == 0) {
		if(buf1 != 0) {
			Display_Char(buf1);
			HAL_GPIO_WritePin(DIGIT_1_GPIO_Port, DIGIT_1_Pin, GPIO_PIN_SET);
		}
	}
	if(dc == 1) {
		if((buf1 != 0) || (buf2 != 0)) {
			Display_Char(buf2);
			HAL_GPIO_WritePin(DIGIT_2_GPIO_Port, DIGIT_2_Pin, GPIO_PIN_SET);
		}
	}
	if(dc == 2) {
		Display_Char(buf3);
		HAL_GPIO_WritePin(DIGIT_3_GPIO_Port, DIGIT_3_Pin, GPIO_PIN_SET);
	}
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	value = 0;
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADCEx_InjectedPollForConversion(&hadc1, 100);
	value = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
	HAL_ADCEx_InjectedStop(&hadc1);
	i = value / 4095 * 331.5;
	if(i > (t + h)) pt = 0;
	if(i < (t - h)) pt = p;
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
