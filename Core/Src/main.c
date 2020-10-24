
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
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


#define DEBUG_PRINT(...) do{\
	uint32_t tick = HAL_GetTick();\
	printf("[%6lu.%03lu]",tick/1000, tick%1000);\
	printf(__VA_ARGS__);\
	}while(0)


typedef struct {
	float x;
	float y;
	float z;
}Coordinate_t;

int8_t buffer[4] = {0,};
enum {
	click_e = 0,
	x_e,
	y_e,
	wheel_e
};
const float sigma = 10.0f;
const float beta = 8/3.0f;
const float rho = 28.0f;

Coordinate_t p = {1.0f, 1.0f, 1.0f};
Coordinate_t d = {0.0f, 0.0f, 0.0f};
Coordinate_t ip = {0.0f, 0.0f, 0.0f};
Coordinate_t id = {0.0f, 0.0f, 0.0f};

const float dt = 0.005f;

Coordinate_t plotCurr = {0.0f, 0.0f, 0.0f};
Coordinate_t plotPrev = {0.0f, 0.0f, 0.0f};
Coordinate_t plotDelta = {0.0f, 0.0f, 0.0f};

static uint32_t delay 	= 30;
static float 	amp 	= 3.0f;

Coordinate_t isometric_projection(Coordinate_t point){
	Coordinate_t ret = {0.0f, 0.0f, 0.0f};
	const static float rot_matrix[3][3] = {
		{sqrt(3)/sqrt(6), 		0, 					-1*sqrt(3)/sqrt(6)},
		{1/sqrt(6), 			2/sqrt(6), 	 		1/sqrt(6)},
		{sqrt(2)/sqrt(6), 		-1*sqrt(2)/sqrt(6), sqrt(2)/sqrt(6)}
	};
	ret.x = rot_matrix[0][0]*point.x + rot_matrix[0][1]*point.y + rot_matrix[0][2]*point.z;
	ret.y = rot_matrix[1][0]*point.x + rot_matrix[1][1]*point.y + rot_matrix[1][2]*point.z;
	ret.z = rot_matrix[2][0]*point.x + rot_matrix[2][1]*point.y + rot_matrix[2][2]*point.z;

	return ret;
}

void iterate(){
	/* Calculate Lorentz attractor */
	d.x = sigma*(p.y - p.x);
	d.y = p.x*(rho - p.z) - p.y;
	d.z = p.x*p.y - beta*p.z;
	d.x *= dt;
	d.y *= dt;
	d.z *= dt;
	p.x += d.x;
	p.y += d.y;
	p.z += d.z;
}


uint32_t RCC_PrintFlag(){
	uint32_t ret = 0;
	DEBUG_PRINT("Print ResetFlag\n");
	if ( __HAL_RCC_GET_FLAG(PWR_FLAG_SB)){
		DEBUG_PRINT("System resumed from STANDBY mode\n");
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == SET)
	{
		DEBUG_PRINT("Software Reset\n"); ret++;
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) == SET)
	{
		DEBUG_PRINT("Power on reset\n"); ret++;
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == SET)
	{
		DEBUG_PRINT("External reset\n"); ret++;
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == SET)
	{
		DEBUG_PRINT("WDT Reset\n"); ret++;
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) == SET)
	{
		DEBUG_PRINT("Window WDT Reset\n"); ret++;
	}
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) == SET)
	{
		DEBUG_PRINT("Low Power Reset\n"); ret++;
	}
	//		if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET) // F4 Usually set with POR
	//		{
	//			printf("Brown-Out Reset\n"); ret++;
	//		}

	/* Clear source Reset Flag */
	__HAL_RCC_CLEAR_RESET_FLAGS();
	return ret;
}

uint32_t read_bkup_reg(){
	static int addr = 11;
	// Read
	uint32_t value = HAL_RTCEx_BKUPRead(&hrtc, addr);
	// Store toggled value
	HAL_RTCEx_BKUPWrite(&hrtc, addr, !value);
	// return
	return !value;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  uint8_t buffer[4];
  buffer[0]=0;//buttons first 3 bits
  buffer[1]=100;//X axis 8bit value signed
  buffer[2]=0;//Y axis 8bit value signed
  buffer[3]=0;//Wheel 8bit value signed
  while(1){
	  USBD_HID_SendReport(&hUsbDeviceFS,buffer,4);
	  HAL_Delay(1000);
  }

  while(1){

  }

  	DEBUG_PRINT("Lorenz Attractor\n");
//	RCC_PrintFlag();
//	uint32_t isrun = read_bkup_reg();
//	DEBUG_PRINT("Read from BKUPReg : %lu\n", isrun);
//	if(!isrun){
//		while(1){
//			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//			HAL_Delay(1500);
//		}
//	}

	DEBUG_PRINT("Waiting for 2seconds... \n");
	for(int i = 0; i < 20; i++){
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		HAL_Delay(100);
	}

	DEBUG_PRINT("Working Test...\n");
	for(int i = 0; i< 20; i++){
		switch(i%4){
		case 0:
			buffer[x_e] = 100;
			buffer[y_e] = 0;
			break;
		case 1:
			buffer[x_e] = 0;
			buffer[y_e] = 100;
			break;
		case 2:
			buffer[x_e] = -100;
			buffer[y_e] = 0;
			break;
		case 3:
			buffer[x_e] = 0;
			buffer[y_e] = -100;
			break;
		}
		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)buffer, 4);
		HAL_Delay(100);
	}
	DEBUG_PRINT("Running Lorenz Attractor...\n");

	while(1)
	{
//		for(int i = 0; i<100000; i++)
		iterate();

		/* Print Axis Value */
#if 0
		DEBUG_PRINT(" x:%6.03f\t y:%6.03f\t z:%6.03f\t\n",p.x,p.y,p.z);
		DEBUG_PRINT("dx:%6.03f\tdy:%6.03f\tdz:%6.03f\t\n",d.x,d.y,d.z);
#endif

		/* Isometric Rotation */
		ip = isometric_projection(p);
#if 0
		DEBUG_PRINT(" x:%6.03f\t y:%6.03f\t z:%6.03f\t\n",ip.x,ip.y,ip.z);
#endif

		/* Plotting */
		plotCurr = ip;
		plotCurr.x *= amp;
		plotCurr.y *= amp;
		plotCurr.z *= amp;
		// To digitize int
		plotDelta.x = roundf(plotCurr.x - plotPrev.x);
		plotDelta.y = roundf(plotCurr.y - plotPrev.y);
		plotDelta.z = roundf(plotCurr.z - plotPrev.z);

		plotPrev.x += plotDelta.x;
		plotPrev.y += plotDelta.y;
		plotPrev.z += plotDelta.z;
#if 0
		DEBUG_PRINT(" x:%6.03f\t y:%6.03f\t z:%6.03f\t\n",plotCurr.x,plotCurr.y,plotCurr.z);
		DEBUG_PRINT("dx:%6.03f\tdy:%6.03f\tdz:%6.03f\t\n",plotDelta.x,plotDelta.y,plotDelta.z);
		printf("\n");
#endif

		/* Send USB Buffer */
		buffer[x_e] = (int8_t) (plotDelta.x);
		buffer[y_e] = (int8_t) (plotDelta.y);
		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *)buffer, 4);

		/* Delay */
		HAL_Delay(delay);

		/* Working LED Indicator */
		static uint32_t i = 0;
		if(!(i++%(100/delay))){ // about 10Hz
			static uint8_t blink[20] = {0,1,0,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1};
			static uint8_t cnt = 0;
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, blink[cnt]);
			cnt++;
			if(sizeof(blink) <= cnt)
				cnt = 0;
//			DEBUG_PRINT("Led Toggle\n");
		}

	}




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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 3 */

  /* USER CODE END RTC_Init 3 */

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 4 */

  /* USER CODE END RTC_Init 4 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
