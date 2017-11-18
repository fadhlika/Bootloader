/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define APPLICATION_ADDRESS 0x08004000
#define PAGE_SIZE 0x400

#define START_ADDRESS ((uint32_t)APPLICATION_ADDRESS)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
typedef void (*pFunction)(void);
pFunction JumpToApplication;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* xConfigure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_CRC_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, (uint8_t*)"\nStart Bootloader\n", 18, 1000);
  HAL_Delay(1000);

  uint8_t *image_buffer;
  uint32_t *image32;

  //Main application image size
  uint32_t NbrOfByte = 0;
  uint32_t NbrOfPage = 0;

  //CRC variable
  uint32_t crcValue;
  uint32_t crcExpected = 0x00000000;

  uint8_t cmd = 0x00;;

  char buf[100]; //Buffer to hold uart transmission

  //Wait for command for 5 seconds
  HAL_UART_Transmit(&huart1, (uint8_t*)"Wait for command\n", 17, 1000);
  HAL_UART_Receive(&huart1, &cmd, 1, 5000);

  switch(cmd){
  	case 0x01: /*Command to flash image---------------------------------------------*/
  		sprintf(buf, "Received command 0x%02x\nWaiting for file\n", cmd);
  		HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 1000);

  		/*Initialized buffer for receiving image file----------------------------------------------*/
  		image_buffer = (uint8_t*)calloc(10240, sizeof(uint8_t));

  		/*Receive image file------------------------------------------------------------------------*/
  		while(HAL_UART_Receive(&huart1, &image_buffer[NbrOfByte++], 1, 2000) != HAL_TIMEOUT);

  		/*Inform file size of image-----------------------------------------------------------------------------*/
  		sprintf(buf, "Received %lu bytes image\n", --NbrOfByte);
  		HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 1000);

  	  /* Convert uint8_t image buffer to uint32_t image buffer--------------------------------------------------------*/
  		/*Initialize buffer for WORD size image*/
  		image32 = (uint32_t*)calloc(NbrOfByte/4, sizeof(uint32_t));

  		int i, j=0;
  		for(i=0; i< NbrOfByte/4; i++)
  		{
  			image32[i] = image_buffer[j] | image_buffer[j+1] << 8 | image_buffer[j+2] << 16 | image_buffer[j+3] << 24;
  			j += 4;
  		}
  		/*Free byte size image buffer*/
  		free(image_buffer);
  		/*--------------------------------------------------------------------------------------------------------------*/

  		/*Receive CRC32 to verify image---------------------------------------------------*/
  		uint8_t crc[4];
  		if(HAL_UART_Receive(&huart1, (uint8_t*)crc, 4, 2000) == HAL_OK){
  			 crcExpected = crc[0] << 24 | crc[1] << 16 | crc[2] << 8 | crc[3];
  		}
  		/*--------------------------------------------------------------------------------------------------------------*/

  		/*Verify image--------------------------------------------------------------------------------------------------*/
  		crcValue = HAL_CRC_Calculate(&hcrc, image32, NbrOfByte/4);

  		if(crcValue != crcExpected)
  		{
  			HAL_UART_Transmit(&huart1, (uint8_t*)"Image corrupted\n", 16, 1000);
  			break;
  		}
  		/*---------------------------------------------------------------------------------------------------------------*/

  		/*Flash image to main memory-------------------------------------------------------------------------------------*/
  		/*Unlock flash memory*/
  		HAL_FLASH_Unlock();

  		/*Calculate number of page*/
  		NbrOfPage = (int)ceil(((((uint32_t)(START_ADDRESS + NbrOfByte)) - START_ADDRESS)/PAGE_SIZE));

  		/*Clear pending flag*/
  		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);


  		/*Erase the flash page*/
  		uint32_t pageError;
  		HAL_StatusTypeDef status;
  		FLASH_EraseInitTypeDef pEraseInit;
  		pEraseInit.Banks = 0;
  		pEraseInit.NbPages = NbrOfPage;
  		pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  		pEraseInit.PageAddress = START_ADDRESS;
  		status = HAL_FLASHEx_Erase(&pEraseInit, &pageError);


  		if(status == HAL_OK){
  			/*Flash*/
  			uint32_t address = START_ADDRESS;
  			int index = 0;
  			while(index < NbrOfByte/4){
  			 sprintf(buf, "Flash 0x%08lx\r\n", address);
  			 HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 1000);

  			 HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, image32[index++]);
  			 address += 4;
  			}
  		}

			free(image32);

  		HAL_FLASH_Lock();
  		/*--------------------------------------------------------------------------------------------------------------*/

  		break;
  }

  /*Jump to application-----------------------------------------------------------------------*/
  HAL_UART_Transmit(&huart1, (uint8_t*)"Load application\n", 18, 1000);

  HAL_UART_DeInit(&huart1);
  HAL_CRC_DeInit(&hcrc);
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
  HAL_RCC_DeInit();
  HAL_DeInit();

  JumpToApplication = (pFunction) (*(__IO uint32_t *)(APPLICATION_ADDRESS + 4));
  __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
  JumpToApplication();
  /*--------------------------------------------------------------------------------------------------------------*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
