/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "dac.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/// Range of digital ADC values
#define ADC_RANGE	4096.0
/// Vref+ ADC
#define ADC_VREF	3.0
/// The size of the array to transfer to USB
#define	SIZE_ARRAY_USB	8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//=============================================================================
/// Flag interrupt SysTick
uint8_t ui8FlagIntSysTick = 0;

//=============================================================================
/// ADC conversion result
uint32_t  ui32ResultConversionADC = 0;
/// ADC conversion result after masking
uint16_t	ui16Result = 0;
//=============================================================================
/// Value in V
float 	  f32Voltage_V = 0;
/// Value in mV
float 	  f32Voltage_mV = 0;
/// Averaging the result in 1 second
float 	  f32VoltageAverage = 0;
/// Accumulation
float		  f32TempVoltageAverage = 0;
/// Counter for 1 sec
uint32_t	ui32CounterVoltageAverage = 0;

//=============================================================================
/// Data array for transmission over USB
uint8_t	ui8TxArrayUSB[SIZE_ARRAY_USB];
//=============================================================================
//=============================================================================
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ConversionDigitalToArray(float input, uint8_t* array);
char ConversionIntegerToChar(uint32_t integer_input);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
	MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	for(i = 0; i < SIZE_ARRAY_USB; i++)
	{
		ui8TxArrayUSB[i] = 0;
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  
  while (1)
  {
		/* USER CODE BEGIN WHILE */
		/// Algorithm of the main program
		/// Check setting the flag every 1 ms 
		if(ui8FlagIntSysTick == 1)
		{
			/// Clear flag interrupt SysTick
			ui8FlagIntSysTick = 0;
			
			/// Start ADC conversion and flow DMA
			if((HAL_ADC_Start(&hadc1)) &&\
				(HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ui32ResultConversionADC,1) == HAL_OK))
			{
				/// Result masking
				ui16Result = (uint16_t)(ui32ResultConversionADC & 0x00000FFF);
				
				/// Start DAC 
				HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
				
				/// Set value DAC
				if(HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,ui16Result) != HAL_OK)
				{
					/// Error message, if DAC not set value
					printf("Error setting value in the DAC \r");
				}
				
				/// Calculate real voltage in V
				f32Voltage_V = (float)((ADC_VREF * ui16Result)/ADC_RANGE);
				
				/// Accumulation result
				f32TempVoltageAverage += f32Voltage_V;
				
				/// Averaging counter increment
				ui32CounterVoltageAverage++;
				
				/// Check averaging counter,
				if(ui32CounterVoltageAverage >= 1000) /// If averaging counter >= 1000, then
				{
					/// Reset averaging counter
					ui32CounterVoltageAverage = 0;
					/// Calculate voltage average value
					f32VoltageAverage = f32TempVoltageAverage/1000;
					
					/// Conversion from volts to millivolts
					f32Voltage_mV = f32VoltageAverage * 1000;
					
					/// Convert voltage to array uint8_t
					ConversionDigitalToArray(f32Voltage_mV,ui8TxArrayUSB);
					
					/// Data transfer through USB-CDC
					CDC_Transmit_FS(ui8TxArrayUSB,SIZE_ARRAY_USB);
					
					/// Print message to SWD Keil
					printf("Input voltage = %4.0f mV\r", f32Voltage_mV);
				}
				
			}
			else
			{
				/// Error message, if ADC error result
				printf("Error ADC conversion \r");
			}
		}// End if(ui8FlagIntSysTick == 1)
    /* USER CODE END WHILE */
		
  }// End While(1)
	/* USER CODE BEGIN 3 */
	
  /* USER CODE END 3 */
}// End Main

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//-------------------------------------------------------------------------------------------------
/**
* @brief  The function is designed to convert digital values into an array of type char.
* @param	input - Float value.
*	@param  pArray - Pointer to array char type.  
* @retval None.	
*/
//-------------------------------------------------------------------------------------------------
void ConversionDigitalToArray(float input, uint8_t* pArray)
{
	uint8_t i = 0;
	uint32_t part_integer = 0;	
	
	uint8_t number_of_digits = 0;	
	uint16_t divider = 0;
	
	// Selection of the whole part (Выделение целой части)
	part_integer = (uint32_t)input;
	
	//
	if((part_integer > 0) && (part_integer < 10))
	{
		divider = 1;
		number_of_digits = 1;
	}
	else if((part_integer >= 10) && (part_integer < 100))
	{
		divider = 10;
		number_of_digits = 2;
	}
	else if((part_integer >= 100) && (part_integer < 1000))
	{
		divider = 100;
		number_of_digits = 3;
	}
	else if((part_integer >= 1000) && (part_integer < 10000))
	{
		divider = 1000;
		number_of_digits = 4;
	}
	
	
	uint32_t temp = 0;
	uint8_t counter_pos = 0;

	// Conversion of numbers to symbols ASCII
	for(i = 0; i < number_of_digits; i++)
	{
		temp = part_integer/divider;	//

		pArray[counter_pos++] = ConversionIntegerToChar(temp);
		
		part_integer = part_integer - temp * divider;
		if(divider != 0)
		{
			divider = divider/10;
		}
	}
	
	pArray[counter_pos++] = ' ';		// Space
	pArray[counter_pos++] = 'm';
	pArray[counter_pos++] = 'V';
	pArray[counter_pos++] = 0x0D;	// "\r"
}
//-------------------------------------------------------------------------------------------------
/**
* @brief  The function is intended to convert an integer number to a character.
* @param	integer_input - Integer value. 		  
* @retval result - ASCII character.
*/
//-------------------------------------------------------------------------------------------------
char ConversionIntegerToChar(uint32_t integer_input)
{
	char result;
	
	switch (integer_input)
	{
		case 0: result = '0'; break;
		case 1: result = '1';	break;
		case 2: result = '2';	break;
		case 3: result = '3';	break;
		case 4: result = '4';	break;
		case 5: result = '5';	break;
		case 6: result = '6';	break;
		case 7: result = '7';	break;
		case 8: result = '8';	break;
		case 9: result = '9';	break;
		default: result = 'N';
	}
	return result;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
