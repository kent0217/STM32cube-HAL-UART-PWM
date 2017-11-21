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
#include "stm32f0xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t receive;
uint8_t send[]="test\n";
int times=0,dir=1,pwm=0;
char * a=0;
char * b=0;
char * c=0;
char * d=0;
#define rx_buffer_size 100

char rx_data[2],rx_buffer[rx_buffer_size],transfer,buffer[rx_buffer_size],rx_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char buff[100];
	sprintf(buff,"%d",receive);

    //HAL_UART_Transmit(&huart1,&receive,sizeof(receive),1000); 
		//HAL_UART_Transmit(&huart1,(uint8_t *)&buff[10],sizeof(buff),1000); 
		if(receive=='A')
		{
			HAL_UART_Transmit(&huart1,&receive,sizeof(receive),1000); 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		}
		if(receive=='B')
		{
			HAL_UART_Transmit(&huart1,&receive,sizeof(receive),1000); 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		}
	  if(receive=='C')
		{
			HAL_UART_Transmit(&huart1,&receive,sizeof(receive),1000); 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		}
			if(receive=='D')
		{
			HAL_UART_Transmit(&huart1,&receive,sizeof(receive),1000); 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		}	
}*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	times++;
  uint8_t i;
	
	if(huart1.Instance==USART1)
 {
	 if(rx_flag==0)
  {
    for(i=0;i<100;i++)
		rx_buffer[i]=0;
  }
   if(rx_flag>rx_buffer_size)	rx_flag=0;
	 if(rx_data[0]!=10)
	 {
	  rx_buffer[rx_flag++]=rx_data[0];
	 }
	 else
	{
		rx_flag=0;
	  transfer=1;
	}
	
				if(transfer)   
		{
		  sprintf(buffer,"%s",rx_buffer);
			a=strstr(buffer, "LED1 ON");
			b=strstr(buffer, "LED2 ON");
			c=strstr(buffer, "LED1 OFF");
			d=strstr(buffer, "LED2 OFF");
		if (a!=NULL )
		{
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET); 
		}
		if(c!=NULL)
		{			
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);	
		}
		if (b!=NULL)
		{
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);		
		}
	  if(d!=NULL)
		{
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
		}		
		HAL_UART_Transmit(&huart1,buffer,sizeof(buffer),1000);			
		transfer=0;
		}		
	HAL_UART_Receive_IT(&huart1,rx_data,1);
 
 }	
}


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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
  //HAL_UART_Receive_IT(&huart1,&receive,sizeof(receive));  	//__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
  HAL_UART_Receive_IT(&huart1,rx_data,1);

/*  test LED
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
HAL_Delay(300);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
HAL_Delay(300);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
HAL_Delay(300);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
HAL_Delay(300);
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {				
	  /**********************UART Transmit TEST**********************/
  //HAL_UART_Transmit(&huart1,send,sizeof(send),100);
  //HAL_Delay(1000);
		/**********************PWM**********************/
		 
		if(dir)
		{	
			pwm++;
			 HAL_UART_Receive_IT(&huart1,rx_data,1);
		}
		else
		{
			pwm--;
			 HAL_UART_Receive_IT(&huart1,rx_data,1);
		}
		if(pwm>200)dir=0;
		if(pwm==0)dir=1;
			 __HAL_TIM_SET_COMPARE(&htim14,TIM_CHANNEL_1,pwm);
			HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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
