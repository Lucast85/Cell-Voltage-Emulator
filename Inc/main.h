/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define E_S_Pin GPIO_PIN_13
#define E_S_GPIO_Port GPIOC
#define EOC_INT_Pin GPIO_PIN_14
#define EOC_INT_GPIO_Port GPIOC
#define CONVST_Pin GPIO_PIN_15
#define CONVST_GPIO_Port GPIOC
#define VOLTAGE_SENSING_Pin GPIO_PIN_0
#define VOLTAGE_SENSING_GPIO_Port GPIOA
#define CURRENT_SENSING_Pin GPIO_PIN_1
#define CURRENT_SENSING_GPIO_Port GPIOA
#define STM32_DAC_Pin GPIO_PIN_4
#define STM32_DAC_GPIO_Port GPIOA
#define LEAKAGE_CURRENT_SENSING_Pin GPIO_PIN_1
#define LEAKAGE_CURRENT_SENSING_GPIO_Port GPIOB
#define AD_CS_Pin GPIO_PIN_2
#define AD_CS_GPIO_Port GPIOB
#define ADBR_P_Pin GPIO_PIN_8
#define ADBR_P_GPIO_Port GPIOE
#define AMBR_P_Pin GPIO_PIN_9
#define AMBR_P_GPIO_Port GPIOE
#define R_LED_Pin GPIO_PIN_14
#define R_LED_GPIO_Port GPIOB
#define G_LED_Pin GPIO_PIN_15
#define G_LED_GPIO_Port GPIOB
#define ADC_CS_Pin GPIO_PIN_8
#define ADC_CS_GPIO_Port GPIOD
#define VCC_CTRL_Pin GPIO_PIN_12
#define VCC_CTRL_GPIO_Port GPIOA
#define DAC_CS_Pin GPIO_PIN_6
#define DAC_CS_GPIO_Port GPIOF
#define REL_VM_Pin GPIO_PIN_5
#define REL_VM_GPIO_Port GPIOB
#define REL_OUT_Pin GPIO_PIN_6
#define REL_OUT_GPIO_Port GPIOB
#define REL_FCM_Pin GPIO_PIN_7
#define REL_FCM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
