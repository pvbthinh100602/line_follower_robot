/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 6024 STMicroelectronics.
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
#include "software_timer.h"
#include "move_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define		ERROR_RANGE		100

#define		CALIB			90
#define 	FORWARD_1 		100
#define 	FORWARD_2 		101
#define 	FORWARD_3 		102
#define 	READYSTEP2		103
#define 	TURNLEFT		104
#define 	TURNRIGHT		105
#define 	FORWARD			106
#define 	BACKWARD		107
#define 	STOP			108
#define 	READYLEFT		109
#define 	READYRIGHT		110
#define 	READYFORWARD	111
#define 	READYBACKWARDS	112
#define 	READYBACKWARDS2	117
#define 	ENDLEFT			113
#define 	ENDRIGHT		114
#define 	ENDWARDS		115
#define 	START			116
#define 	ENDBACK			118

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t status = CALIB;
uint8_t temp_status = FORWARD_1;
//uint8_t temp_status = FORWARD_1;
uint8_t step = 0;
uint8_t count = 0;
uint8_t arr[50] = "FRFLFRFRFFRFFR\0";
uint8_t index = 0;
uint16_t sensor_value[5];
uint16_t sensor_calib[5];
int sensor_buffer = 0;
int led_count = 0;
int led_cycle = 25;
int button_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void buttonScan();
void sensorScan();
void ledBlink();
uint8_t checkLine();
uint8_t checkStatus();
void sensorCalib();
void lineSet();
void test();
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setTimer(1,10);
  while (1)
  {
	  if(timer_flag[1]){
		  setTimer(1,10);
		  buttonScan();
//		  sensorScan();
		  checkLine();
		  lineSet();
//		  forward();
//		  backwards();
		  ledBlink();
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 144-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, LED_DEBUG_Pin|IN1_DC1_Pin|IN2_DC1_Pin|IN1_DC2_Pin
                          |IN2_DC2_Pin|IN1_DC3_Pin|IN2_DC3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_DC4_Pin|IN2_DC4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_KEY_Pin */
  GPIO_InitStruct.Pin = USER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_DEBUG_Pin IN1_DC1_Pin IN2_DC1_Pin IN1_DC2_Pin
                           IN2_DC2_Pin IN1_DC3_Pin IN2_DC3_Pin */
  GPIO_InitStruct.Pin = LED_DEBUG_Pin|IN1_DC1_Pin|IN2_DC1_Pin|IN1_DC2_Pin
                          |IN2_DC2_Pin|IN1_DC3_Pin|IN2_DC3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_DC4_Pin IN2_DC4_Pin */
  GPIO_InitStruct.Pin = IN1_DC4_Pin|IN2_DC4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef * htim ){
	timerRun(1);
}

void buttonScan(){
	if(HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == 1) button_count++;
	else button_count = 0;
}


void sensorScan(){
	HAL_ADC_Start_DMA(&hadc1, sensor_value, 5);
	sensor_buffer = 0;
	for(int i = 0; i < 5; i++){
		sensor_buffer = sensor_buffer << 1;
//		if(sensor_value[i] > sensor_calib[i] - ERROR_RANGE && sensor_value[i] < sensor_calib[i] + ERROR_RANGE) sensor_buffer++;
		if((sensor_value[i] > sensor_calib[i] - ERROR_RANGE) && (sensor_value[i] < sensor_calib[i] + ERROR_RANGE)) sensor_buffer++;
	}
}

void ledBlink(){
	led_count++;
	if(led_count >= led_cycle) {
		led_count = 0;
		HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
	}
}

//uint8_t checkLine(){
////	uint8_t check;
////	check = (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0) + (HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0) + (HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0)
////			+ (HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0) + (HAL_GPIO_ReadPin(S5_GPIO_Port, S5_Pin) == 0);
////	return sensor_buffer == 0b111 || sensor_buffer == 0b101 || sensor_buffer == 0b110 || sensor_buffer == 0b011;
//	uint8_t result = 0;
//		for(int i = 0; i < 5; i++){
//			if((sensor_buffer & (1u << i)) != 0) result++;	// i là 0 1 2 3 4, ex: abcde & 00100 = 00c00
//			//&& trả v�? true or false, & so bitwise rồi chuyển từ nhị phân lại thập phân
//		}
//	return result;
//}

#define LINE_CENTER	0
#define LINE_CROSS	1
#define LINE_ERROR	2
#define LINE_LEFT	3
#define LINE_RIGHT	4

uint8_t line_code = 0;
uint8_t checkLine(){
	sensorScan();
	switch (sensor_buffer) {
		case 0b00000:
			line_code = LINE_ERROR;
			break;
		case 0b11111:
		case 0b11110:
		case 0b01111:
		case 0b01010:
		case 0b11011:
			line_code = LINE_CROSS;
			break;
		case 0b00100:
		case 0b01110:
			line_code = LINE_CENTER;
			break;

		case 0b11100:
		case 0b11000:
		case 0b10000:
		case 0b01100:
		case 0b01000:
			line_code = LINE_LEFT;
			break;

		case 0b00111:
		case 0b00011:
		case 0b00001:
		case 0b00110:
		case 0b00010:
			line_code = LINE_RIGHT;
			break;
		default:
			line_code = LINE_ERROR;
			break;
	}
	return 0;
}

uint8_t checkStatus(){
	index++;
	if(arr[index-1] == '\0')
		return STOP;
	if(arr[index-1] == 'L')
		return TURNLEFT;
	if(arr[index-1] == 'R')
		return TURNRIGHT;
	if(arr[index-1] == 'F')
		return FORWARD;
	if(arr[index-1] == 'B')
		return READYBACKWARDS;
	return 0;
}

void sensorCalib(){
	for(int i = 0; i < 5; i++){
		sensor_calib[i] = sensor_value[i];
	}
}

void followLine(){
	switch (line_code) {
		case LINE_ERROR:
			forward();
			break;
		case LINE_CENTER:
			forward();
			break;
		case LINE_RIGHT:
			rotateRight();
			break;
		case LINE_LEFT:
			rotateLeft();
			break;
		case LINE_CROSS:
			stop();
			break;
		default:
			break;
	}
}

int status_follow = 0;
int followLineUntilCross(){
//	 status = 1
//	        count = 0
//
//	        while True:
//	            line_state = self._line_sensor.check()
//
//	            if status == 1:
//	                if line_state != LINE_CROSS:
//	                    status = 2
//	            elif status == 2:
//	                if line_state == LINE_CROSS:
//	                    count = count + 1
//	                    if count == 2:
//	                        break
//
//	            await self.follow_line(True, line_state)
//
//	            if status == 2 and count == 1:
//	                await asleep_ms(20)
//	            else:
//	                await asleep_ms(10)
//
//	        #await self.forward_for(0.1, unit=SECOND) # to pass cross line a bit
//	        await self.stop_then(then)
	if(status_follow == 0){
		if(line_code != LINE_CROSS) status_follow = 1;
	} else if(status_follow == 1){
		if(line_code == LINE_CROSS) {
			stop();
			return 1;
		}
	}
	followLine();
	return 0;
}

int status_turn_left = 0;
int turnLeftUntilLine(){
	rotateLeft();
	if(status_turn_left == 0){
		if(line_code != LINE_CENTER) status_turn_left = 1;
	} else if(status_turn_left == 1){
		if(line_code == LINE_LEFT) {
			status_turn_left = 0;
			stop();
			return 1;
		}
	}
	return 0;
}

int status_turn_right = 0;
int turnRightUntilLine(){
	rotateRight();
	if(status_turn_right == 0){
		if(line_code != LINE_CENTER) status_turn_right = 1;
	} else if(status_turn_right == 1){
		if(line_code == LINE_RIGHT) {
			status_turn_right = 0;
			stop();
			return 1;
		}
	}

	return 0;
}

#define GET_STATUS	21
void lineSet(){
	switch(status) {
		case CALIB:
			if(button_count == 100){
				sensorCalib();
				led_cycle = 100;
				status = START;
			}
			break;
		case GET_STATUS:
			if(count > 0) count--;
			if(count == 0){
				status = checkStatus();
			}
			break;
		case START:
			if(button_count == 1){
				index = 0;
				status = GET_STATUS;
//				status = FOLLOW;
			}
			break;
		case FORWARD:
			if(followLineUntilCross() == 1) {
				status = GET_STATUS;
				count = 100;
			}
//			if(checkLine() == LINE_CENTER)
//			forward();
//			if(checkLine() == LINE_CENTER){
//				status = ENDWARDS;
//				stop();
//				count = 20;
//			}
			break;
		case READYBACKWARDS:
			backwards();
			if(checkLine() >= 3){
				status = READYBACKWARDS2;
			}
			break;
		case READYBACKWARDS2:
			backwards();
			if(checkLine() < 2){
				status = BACKWARD;
				stop();
			}
			break;
		case BACKWARD:
			backwards();
			if(checkLine() >= 3){
				status = ENDBACK;
				stop();
				count = 20;
			}
			break;
		case ENDBACK:
			count--;
			if(count <= 0){
				if(checkLine() >= 3){
					status = ENDWARDS;
					stop();
					count = 20;
				}
				if(checkLine() < 3){
					status = FORWARD;
				}
			}
			break;
		case ENDWARDS:
			count--;
			if(count <= 0){
				if(checkLine() >= 3)
					forward();
				else{
					stop();
					status = checkStatus();
				}
			}
			break;
		case TURNLEFT:
//			left();
//			if(sensor_buffer == 0b10000){
//				status = ENDLEFT;
//			}
			if(turnLeftUntilLine() == 1) {
				status = GET_STATUS;
				count = 100;
			}
			break;
		case ENDLEFT:
			left();
			if(sensor_buffer == 0b00100){
				stop();
				status = checkStatus();
			}
			break;
		case TURNRIGHT:
//			right();
//			if(sensor_buffer == 0b00001){
//				status = ENDRIGHT;
//			}
			if(turnRightUntilLine() == 1) {
				status = GET_STATUS;
				count = 100;
			}
			break;
		case ENDRIGHT:
			right();
			if(sensor_buffer == 0b00100){
				stop();
				status = checkStatus();
			}
			break;
		case STOP:
			stop();
			break;
		default:
			status = FORWARD_1;
			break;
	}
}

//void lineSet(){
//	switch(status) {
//		case FOLLOW:
//			followLine();
//			break;
//		case CALIB:
//			if(button_count == 100){
//				sensorCalib();
//				led_cycle = 100;
//				status = START;
//			}
//			break;
//		case START:
//			if(button_count == 1){
//				index = 0;
////				status = checkStatus();
//				status = FOLLOW;
//			}
//			break;
//		case FORWARD:
//			if(checkLine() == LINE_CENTER)
//			forward();
//			if(checkLine() == LINE_CENTER){
//				status = ENDWARDS;
//				stop();
//				count = 20;
//			}
//			break;
//		case READYBACKWARDS:
//			backwards();
//			if(checkLine() >= 3){
//				status = READYBACKWARDS2;
//			}
//			break;
//		case READYBACKWARDS2:
//			backwards();
//			if(checkLine() < 2){
//				status = BACKWARD;
//				stop();
//			}
//			break;
//		case BACKWARD:
//			backwards();
//			if(checkLine() >= 3){
//				status = ENDBACK;
//				stop();
//				count = 20;
//			}
//			break;
//		case ENDBACK:
//			count--;
//			if(count <= 0){
//				if(checkLine() >= 3){
//					status = ENDWARDS;
//					stop();
//					count = 20;
//				}
//				if(checkLine() < 3){
//					status = FORWARD;
//				}
//			}
//			break;
//		case ENDWARDS:
//			count--;
//			if(count <= 0){
//				if(checkLine() >= 3)
//					forward();
//				else{
//					stop();
//					status = checkStatus();
//				}
//			}
//			break;
//		case TURNLEFT:
//			left();
////			if(HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 1)
//			if(sensor_buffer == 0b10000){
//				status = ENDLEFT;
//			}
//			break;
//		case ENDLEFT:
//			left();
//			if(sensor_buffer == 0b00100){
//				stop();
//				status = checkStatus();
//			}
//			break;
//		case TURNRIGHT:
//			right();
//			if(sensor_buffer == 0b00001){
//				status = ENDRIGHT;
//			}
//			break;
//		case ENDRIGHT:
//			right();
//			if(sensor_buffer == 0b00100){
//				stop();
//				status = checkStatus();
//			}
//			break;
//		case STOP:
//			stop();
//			break;
//		default:
//			status = FORWARD_1;
//			break;
//	}
//}

void test(){
	dc1Forward(50);
	HAL_Delay(1000);
	dc2Forward(50);
	HAL_Delay(1000);
	dc3Forward(50);
	HAL_Delay(1000);
	dc4Forward(50);
	HAL_Delay(1000);
	stop();
	HAL_Delay(1000);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
