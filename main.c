/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void Config_LCD(uint8_t clear_CMD, uint8_t function_SET,uint8_t display_Ctrl, uint8_t return_CMD);
void Clear_LCD(uint8_t cmd);
void Function_Set(uint8_t cmd);
void Display_Control(uint8_t cmd);
void Return_Home(uint8_t cmd);
void Write_LCD_Data(char data[]);
void Set_Cursor(uint8_t cmd);
void Blink_Cursor(uint8_t cmd);
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
  /* USER CODE BEGIN 2 */
  Function_Set(0x38);//8-bit data, 2-lines
  Clear_LCD(0x01);//clear the display
  Display_Control(0x0c); //Enable the display
  Return_Home(0x02); //Set cursor at address 0x00, line-1
  char * msg = "ENSA MARRAKECH.";
  Write_LCD_Data(msg); //Write data
  Set_Cursor(0xC0);// Set cursor at line-2
  Write_LCD_Data("I'm home Buddies");//write data to the screen
  HAL_Delay(2000);
  Set_Cursor(0x02); //Set cursor to home. Equivalent to Return_Home(cmd)
  Write_LCD_Data("Patricia Aboraa"); //Write data
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|RW_Pin|E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin D3_Pin
                           D4_Pin D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin RW_Pin E_Pin */
  GPIO_InitStruct.Pin = RS_Pin|RW_Pin|E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Write_LCD_Data(char data[]){
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, SET);
	HAL_Delay(1);
	int i = 0;
	while(data[i] != '\0'){
		GPIOA->ODR =(uint32_t)data[i];
		HAL_GPIO_WritePin(E_GPIO_Port, E_Pin,SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(E_GPIO_Port, E_Pin,RESET);
//		HAL_Delay(1);
		i++;
	}
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	HAL_Delay(1);
};
void Clear_LCD(uint8_t cmd){
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	GPIOA->ODR = cmd;
	HAL_Delay(1);
}

void Function_Set(uint8_t cmd){
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	HAL_Delay(1);
	GPIOA->ODR = cmd;
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, RESET);
}

void Display_Control(uint8_t cmd){
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	HAL_Delay(1);
	GPIOA->ODR = cmd;
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, RESET);

}

void Return_Home(uint8_t cmd){
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	HAL_Delay(1);
	GPIOA->ODR = cmd;
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, RESET);
}

void Set_Cursor(uint8_t cmd){
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	HAL_Delay(1);
	GPIOA->ODR = cmd;
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, RESET);
}

void Blink_Cursor(uint8_t cmd){
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, RESET);
	HAL_Delay(1);
	GPIOA->ODR = cmd;
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(E_GPIO_Port, E_Pin, RESET);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
