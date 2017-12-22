/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
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
#include "main.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "sdadc.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#define VDD_mV															3340.0
#define CONVERT_RAW_12bitDATA_TO_mV(value) 	(value*VDD_mV/4095.0)
#define CONVERT_mV_TO_RAW_12bitDATA(value)	(uint16_t)(value*4095/VDD_mV)
#define CONVERT_RAW_16bitDATA_TO_mV(value) 	(value*VDD_mV/32767.0)
#define TWOSCOMPLEMENT(value)							 	((~value)+1)

/* PID parameters */
#define PID_PARAM_KP        0.01     	/* Proportional */
#define PID_PARAM_KI        0.025   	/* Integral */
#define PID_PARAM_KD        0      /* Derivative */

#define V_MEAS_DIVIDER_KP		1.51			/* multiplication factor the voltage mearurement divider (R67, R69) */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t G_LED_sts;
uint8_t ADCval;
uint32_t SARADCval, SDADCval;
float DACvalue_mV;
uint16_t DACvalue_u16;
uint16_t ADCinDMArawValues[2];
// PID variables
float pid_error;
arm_pid_instance_f32 PID;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	printf("SWD TERMINAL enabled!!!\n");
	/*-------------PID initialization---------------------------*/
	PID.Kp = PID_PARAM_KP;	/* Proportional */
	PID.Ki = PID_PARAM_KI;	/* Integral */
	PID.Kd = PID_PARAM_KD;	/* Derivative */ 
	/* Initialize PID system */
	arm_pid_init_f32(&PID, 1);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_COMP2_Init();
  MX_DAC1_Init();
  MX_SDADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
	// shut down all leds
	HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R_LED_GPIO_Port, R_LED_Pin, GPIO_PIN_SET);
	// close mosfet Q1 VCC-CTRL and supply Vcc=5V to the board
	HAL_GPIO_WritePin(VCC_CTRL_GPIO_Port, VCC_CTRL_Pin, GPIO_PIN_RESET);

	// open output relay
	HAL_GPIO_WritePin(REL_OUT_GPIO_Port, REL_OUT_Pin, GPIO_PIN_RESET);
	// open voltage measurement relay
	HAL_GPIO_WritePin(REL_VM_GPIO_Port, REL_VM_Pin, GPIO_PIN_RESET);
	
	// close output relay
	HAL_GPIO_WritePin(REL_OUT_GPIO_Port, REL_OUT_Pin, GPIO_PIN_SET);
	// close voltage measurement relay
	HAL_GPIO_WritePin(REL_VM_GPIO_Port, REL_VM_Pin, GPIO_PIN_SET);


	// open fine current measurement relay
	HAL_GPIO_WritePin(REL_FCM_GPIO_Port, REL_FCM_Pin, GPIO_PIN_RESET);
	
	// Calibration of SAR ADCs
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
		printf("ERROR: SAR ADC not calibrated!");
    Error_Handler();
  }
	
	// Calibration of SD ADCs
	HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1);
	// wait up to 2 seconds for the end of calibration
	if(HAL_SDADC_PollForCalibEvent(&hsdadc1, 2000) != HAL_OK ){
		printf("ERROR: SDADC not calibrated!");
		Error_Handler();
	}
	// Select regular channel and enable/disable continuous mode
	HAL_SDADC_ConfigChannel(&hsdadc1, SDADC_CHANNEL_7, SDADC_CONTINUOUS_CONV_OFF);
	
	// DAC initialization:
	HAL_DAC_Init(&hdac1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	
	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
 {
		//*********** BLINK ACTIVITY LED ***************
		HAL_Delay(300);
		HAL_GPIO_TogglePin(G_LED_GPIO_Port, G_LED_Pin);
	 
		//*********** TEST ADC *************************
	/*
		// test of SAR ADC by using polling mode
		// Activate the ADC peripheral and start conversions using function
		HAL_ADC_Start(&hadc1);
		// Wait for ADC conversion completion using function
		HAL_ADC_PollForConversion(&hadc1, 100);
		// Retrieve conversion results using function 
		SARADCval = HAL_ADC_GetValue(&hadc1);
		// Stop conversion and disable the ADC peripheral using function
		HAL_ADC_Stop(&hadc1);
		// print the ADC converted value
		printf("RAW SAR ADC value is: %d\n", SARADCval);
		printf("Converted SAR ADC value is: %d\n", CONVERT_RAW_12bitDATA_TO_mV(SARADCval));
	*/
		//*********** TEST ADC in DMA mode *************************
		//**********************************************************
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCinDMArawValues, 2);
		HAL_Delay(100);
		HAL_ADC_Stop_DMA(&hadc1);
		for(uint8_t i=0; i<hadc1.Init.NbrOfConversion; i++){
			printf("ADC in DMA mode value #%d = %2.2f mV\n", i, CONVERT_RAW_12bitDATA_TO_mV(ADCinDMArawValues[i]));
		}
		printf("RAW Vmeas= %d\n", ADCinDMArawValues[0]);
	 
		//********** test Sigma-Delta ADC using polling mode *****************************
		//********************************************************************************
		// Start regular conversion
		HAL_SDADC_Start(&hsdadc1);
		// In polling mode, use HAL_SDADC_PollForConversion to detect the end of regular conversion.
		HAL_SDADC_PollForConversion(&hsdadc1, 1000);
		
		// Select trigger for regular conversion
		// HAL_SDADC_SelectRegularTrigger(&hsdadc1, SDADC_SOFTWARE_TRIGGER);
		// Get value of regular conversion using
		SDADCval = HAL_SDADC_GetValue(&hsdadc1);
//		printf("SDADC raw value is = %d \n", SDADCval);
//		printf("SDADC value is = %u mV\n", CONVERT_RAW_16bitDATA_TO_mV(SDADCval));
		// Stop regular conversion
		HAL_SDADC_Stop(&hsdadc1);

	/*************** TEST DAC *************************/
	/***************************************************
	DACvalue=DACvalue+10;
  DACvalue = DACvalue % 4096;
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DACvalue);
	//HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
	printf("DAC RAW Value set is: %d\n", HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_1));
	printf("DAC analog Value set is: %d\n", CONVERT_RAW_12bitDATA_TO_mV(DACvalue));
	*/
	
	/*************** TEST PID *************************/
	/* Calculate error (read voltage - voltage want) */
	pid_error =  CONVERT_RAW_12bitDATA_TO_mV( ADCinDMArawValues[0] ) *V_MEAS_DIVIDER_KP - CONVERT_RAW_16bitDATA_TO_mV(SDADCval)*10.0;
	printf("%2.2f (pid_error) = %2.2f (Vmeas) - %2.2f (AMBR*10)\n", pid_error, \
		CONVERT_RAW_12bitDATA_TO_mV( ADCinDMArawValues[0] )* V_MEAS_DIVIDER_KP, CONVERT_RAW_16bitDATA_TO_mV(SDADCval)*10.0);
	/* Calculate PID here, argument is error */
	/* Output data will be returned, I will use it as input voltage */
	DACvalue_mV = arm_pid_f32(&PID, pid_error);
	DACvalue_u16 = CONVERT_mV_TO_RAW_12bitDATA(DACvalue_mV);
	
	printf("DACvalue_u16 (RAW) is: %d \n", DACvalue_u16);
	// check for overflow of DAC value
	if(DACvalue_u16 > 4095){
		DACvalue_u16 = 4095;
	}
	else if(DACvalue_u16 < 0){
		DACvalue_u16 = 0;
	}
	
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DACvalue_u16);
	//HAL_DAC_GetValue(&hdac1, DAC_CHANNEL_1)
	printf("DACvalue (RAW) in mV is: %2.2f\n",CONVERT_RAW_12bitDATA_TO_mV(DACvalue_mV));
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC1|RCC_PERIPHCLK_SDADC;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV48;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PCLK2_DIV2;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
	// shut down green led
	HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_SET);
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
		// toggle red led to show error state!
		HAL_GPIO_TogglePin(R_LED_GPIO_Port, R_LED_Pin);
		HAL_Delay(100);
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
