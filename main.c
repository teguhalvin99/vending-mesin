/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int tik;
extern char tss;


typedef enum state {st0,st500,st1000,stopen,stkembali} state_t;

state_t presentState;
state_t nextState;

unsigned char bouncing500	=0xFF;
uint32_t Timeout_loop500 	= 0;
uint32_t Timeout_value500 	= 600;

unsigned char bouncing1000	=0xFF;
uint32_t Timeout_loop1000 	= 0;
uint32_t Timeout_value1000 	= 600;

unsigned char bouncingpros	=0xFF;
uint32_t Timeout_looppros 	= 0;
uint32_t Timeout_valuepros 	= 600;

unsigned char bouncingbat	=0xFF;
uint32_t Timeout_loopbat 	= 0;
uint32_t Timeout_valuebat 	= 600;


unsigned char over1000=0,over500=0;
unsigned char flagbat=0;

void kembali500(char on){
	if(on==1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}
}

void kembali1000(char on){
	if(on==1){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}
}

void opendrink(char on){
	if(on==1){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		}
}
const char add=4;
void cekSensor(){
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)== GPIO_PIN_RESET && (Timeout_loop500++<=Timeout_value500)){
		bouncing500=(bouncing500<<1);
	}

	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)== GPIO_PIN_RESET && (Timeout_loop500++>Timeout_value500)){
		if(Timeout_loop500>=Timeout_value500+50){
			Timeout_loop500=Timeout_value500+1;
			HAL_GPIO_TogglePin(User_LED_GPIO_Port,User_LED_Pin);
			HAL_UART_Transmit(&huart1,(uint8_t*)"\n\rErr: 005-Sensor3",18,10);
		}
	}

	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)== GPIO_PIN_SET && Timeout_loop500<=Timeout_value500){
		bouncing500 = (bouncing500<<1)|1;
	}

	else{
		Timeout_loop500=0;
		bouncing500=0xFF;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)== GPIO_PIN_RESET && (Timeout_loop1000++<=Timeout_value1000)){
		bouncing1000=(bouncing1000<<1);
	}

	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)== GPIO_PIN_RESET && (Timeout_loop1000++>Timeout_value1000)){
		if(Timeout_loop1000>=Timeout_value1000+50){
			Timeout_loop1000=Timeout_value1000+1;
			HAL_GPIO_TogglePin(User_LED_GPIO_Port,User_LED_Pin);
			HAL_UART_Transmit(&huart1,(uint8_t*)"\n\rErr: 005-Sensor3",18,10);
		}
	}

	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)== GPIO_PIN_SET && Timeout_loop1000<=Timeout_value1000){
		bouncing1000 = (bouncing1000<<1)|1;
	}

	else{
		Timeout_loop1000=0;
		bouncing1000=0xFF;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)== GPIO_PIN_RESET && (Timeout_loopbat++<=Timeout_valuebat)){
		bouncingbat=(bouncingbat<<1);
	}

	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)== GPIO_PIN_RESET && (Timeout_loopbat++>Timeout_valuebat)){
		if(Timeout_loopbat>=Timeout_valuebat+50){
			Timeout_loopbat=Timeout_valuebat+1;
			HAL_GPIO_TogglePin(User_LED_GPIO_Port,User_LED_Pin);
			HAL_UART_Transmit(&huart1,(uint8_t*)"\n\rErr: 005-Sensor3",18,10);
		}
	}

	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)== GPIO_PIN_SET && Timeout_loopbat<=Timeout_valuebat){
		bouncingbat = (bouncingbat<<1)|1;
	}

	else{
		Timeout_loopbat=0;
		bouncingbat=0xFF;
	}


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)== GPIO_PIN_RESET && (Timeout_looppros++<=Timeout_valuepros)){
		bouncingpros=(bouncingpros<<1);
	}

	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)== GPIO_PIN_RESET && (Timeout_looppros++>Timeout_valuepros)){
		if(Timeout_looppros>=Timeout_valuepros+50){
			Timeout_looppros=Timeout_valuepros+1;
			HAL_GPIO_TogglePin(User_LED_GPIO_Port,User_LED_Pin);
			HAL_UART_Transmit(&huart1,(uint8_t*)"\n\rErr: 005-Sensor3",18,10);
		}
	}

	else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)== GPIO_PIN_SET && Timeout_looppros<=Timeout_valuepros){
		bouncingpros = (bouncingpros<<1)|1;
	}

	else{
		Timeout_looppros=0;
		bouncingpros=0xFF;
	}
}

void dropkoin(void){
	int out;
	if(over1000==1){
		HAL_UART_Transmit(&huart1,(uint8_t*)"Kembalian -> 1000 Rupiah\n\n",27,10);
		for(out=0;out<=2000;out=out+add){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_Delay(2);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		over1000=0;
	}
	else if(over500==1){
		HAL_UART_Transmit(&huart1,(uint8_t*)"Kembalian -> 500 Rupiah\n\n",27,10);
		for(out=0;out<=2000;out=out+add){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_Delay(2);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		over500=0;
	}
}

void dropcandy(void){
	int out;
	HAL_UART_Transmit(&huart1,(uint8_t*)"---GET YOUR CANDY--- \n\n\n",25,40);
	for(out=10;out>=0;out=out-add){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(200);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1,(uint8_t*)"==== Vending Machine Simulation V1.0====\n\nMasukkan Uang :\n",60,40);
}


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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

  HAL_UART_Transmit(&huart1,(uint8_t*)"==== Vending Machine Simulation V1.0====\n\nMasukkan Uang :\n",60,40);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(tss==1){
		switch(presentState){

			case st0:
				cekSensor();
				if(bouncing500==0x01){
					HAL_UART_Transmit(&huart1,(uint8_t*)"Anda memasukkan 500 Rupiah\n",28,10);
					nextState=st500;
				}
				else if(bouncing1000==0x01){
					HAL_UART_Transmit(&huart1,(uint8_t*)"Anda memasukkan 1000 Rupiah\n",29,10);
					nextState=st1000;
				}

			break;

			case st500:
				cekSensor();
				if(bouncing500==0x01){
					HAL_UART_Transmit(&huart1,(uint8_t*)"Anda memasukkan 500 Rupiah\n",28,10);
					nextState=st1000;
				}
				else if(bouncing1000==0x01){
					HAL_UART_Transmit(&huart1,(uint8_t*)"Anda memasukkan 1000 Rupiah\n",29,10);
					nextState=stkembali;
					over500=1;
				}
				else if(bouncingbat==0x01){
					HAL_UART_Transmit(&huart1,(uint8_t*)"Anda Membatalkan Transaksi\n\n",30,10);
					nextState=stkembali;
					over500=1;
					flagbat=1;
				}
			break;

			case st1000:
				cekSensor();
				if(bouncing500==0x01){
					HAL_UART_Transmit(&huart1,(uint8_t*)"Anda memasukkan 500 Rupiah\n",28,10);
					nextState=stkembali;
					over500=1;
				}
				else if(bouncing1000==0x01){
					HAL_UART_Transmit(&huart1,(uint8_t*)"Anda memasukkan 1000 Rupiah\n",29,10);
					nextState=stkembali;
					over1000=1;
				}
				else if(bouncingbat==0x01){
					HAL_UART_Transmit(&huart1,(uint8_t*)"Anda Membatalkan Transaksi\n\n",30,10);
					nextState=stkembali;
					over1000=1;
					flagbat=1;
				}
				else if(bouncingpros==0x01){
					nextState=stopen;
				}
			break;

			case stkembali:
				dropkoin();
				if(flagbat==1){
					nextState=st0;
					flagbat=0;
				}
				else if(flagbat==0){
					nextState=st1000;
				}
			break;

			case stopen:
				dropcandy();
				nextState=st0;
			break;



		}
		presentState=nextState;

		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		tss=0;
	}
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_LED_Pin */
  GPIO_InitStruct.Pin = User_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(User_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
