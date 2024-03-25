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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define		ERROR_RANGE		50

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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t status = START;
uint8_t temp_status = FORWARD_1;
uint8_t step = 0;
uint8_t count = 0;
uint8_t arr[50] = "FRFLBRBL\0";
uint8_t index = 0;
uint16_t sensor_value[3];
uint16_t sensor_calib[3];
int sensor_buffer = 0;
int led_count = 0;
int led_cycle = 25;
int button_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void button_scan();
void sensor_scan();
void led_blink();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void button_scan(){
	if(HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == 1) button_count++;
	else button_count = 0;
}

void sensor_scan(){
	HAL_ADC_Start_DMA(&hadc1, sensor_value, 3);
	sensor_buffer = 0;
	for(int i = 0; i < 3; i++){
		sensor_buffer = sensor_buffer << 1;
		if(sensor_value[i] > sensor_calib[i] - ERROR_RANGE && sensor_value[i] < sensor_calib[i] + ERROR_RANGE) sensor_buffer++;
	}
}

void led_blink(){
	led_count++;
	if(led_count == led_cycle) {
		led_count = 0;
		HAL_GPIO_TogglePin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
	}
}

uint8_t speed_duty_cycle = 0;
void set_speed(uint8_t dc, uint8_t duty_cycle) {
	speed_duty_cycle = duty_cycle;
	switch (dc){
	case 1:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed_duty_cycle);
		break;
	case 2:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed_duty_cycle);
		break;
	case 3:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed_duty_cycle);
		break;
	case 4:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed_duty_cycle);
		break;
	}
}

void dc1_forward(uint8_t duty_cycle){ //tiến
  HAL_GPIO_WritePin(IN1_DC1_GPIO_Port, IN1_DC1_Pin, 1);
  HAL_GPIO_WritePin(IN2_DC1_GPIO_Port, IN2_DC1_Pin, 0);
  set_speed(1, duty_cycle);
}

void dc1_backwards(uint8_t duty_cycle){ //lùi
  HAL_GPIO_WritePin(IN1_DC1_GPIO_Port, IN1_DC1_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC1_GPIO_Port, IN2_DC1_Pin, 1);
  set_speed(1, duty_cycle);
}

void dc1_stop(){
  HAL_GPIO_WritePin(IN1_DC1_GPIO_Port, IN1_DC1_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC1_GPIO_Port, IN2_DC1_Pin, 0);
}

void dc2_forward(uint8_t duty_cycle){ //tiến
  HAL_GPIO_WritePin(IN1_DC2_GPIO_Port, IN1_DC2_Pin, 1);
  HAL_GPIO_WritePin(IN2_DC2_GPIO_Port, IN2_DC2_Pin, 0);
  set_speed(2, duty_cycle);
}

void dc2_backwards(uint8_t duty_cycle){ //lùi
  HAL_GPIO_WritePin(IN1_DC2_GPIO_Port, IN1_DC2_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC2_GPIO_Port, IN2_DC2_Pin, 1);
  set_speed(2, duty_cycle);
}

void dc2_stop(){
  HAL_GPIO_WritePin(IN1_DC2_GPIO_Port, IN1_DC2_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC2_GPIO_Port, IN2_DC2_Pin, 0);
}

void dc3_forward(uint8_t duty_cycle){ //tiến
  HAL_GPIO_WritePin(IN1_DC3_GPIO_Port, IN1_DC3_Pin, 1);
  HAL_GPIO_WritePin(IN2_DC3_GPIO_Port, IN2_DC3_Pin, 0);
  set_speed(3, duty_cycle);
}

void dc3_backwards(uint8_t duty_cycle){ //lùi
  HAL_GPIO_WritePin(IN1_DC3_GPIO_Port, IN1_DC3_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC3_GPIO_Port, IN2_DC3_Pin, 1);
  set_speed(3, duty_cycle);
}

void dc3_stop(){
  HAL_GPIO_WritePin(IN1_DC3_GPIO_Port, IN1_DC3_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC3_GPIO_Port, IN2_DC3_Pin, 0);
}

void dc4_forward(uint8_t duty_cycle){ //tiến
  HAL_GPIO_WritePin(IN1_DC4_GPIO_Port, IN1_DC4_Pin, 1);
  HAL_GPIO_WritePin(IN2_DC4_GPIO_Port, IN2_DC4_Pin, 0);
  set_speed(4, duty_cycle);
}

void dc4_backwards(uint8_t duty_cycle){ //lùi
  HAL_GPIO_WritePin(IN1_DC4_GPIO_Port, IN1_DC4_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC4_GPIO_Port, IN2_DC4_Pin, 1);
  set_speed(4, duty_cycle);
}

void dc4_stop(){
  HAL_GPIO_WritePin(IN1_DC4_GPIO_Port, IN1_DC4_Pin, 0);
  HAL_GPIO_WritePin(IN2_DC4_GPIO_Port, IN2_DC4_Pin, 0);
}

void stop(){
	dc1_stop();
	dc2_stop();
	dc3_stop();
	dc4_stop();
}

void forward(){
	dc1_forward(60);
	dc2_forward(60);
	dc3_forward(60);
	dc4_forward(60);
}

void backwards(){
	dc1_backwards(60);
	dc2_backwards(60);
	dc3_backwards(60);
	dc4_backwards(60);
}

void front_left(){
	dc2_forward(60);
	dc3_forward(60);
	dc1_stop();
	dc4_stop();
//	stop();
}

void front_right(){
	dc1_forward(60);
	dc4_forward(60);
	dc2_stop();
	dc3_stop();
//	stop();
}
void back_left(){
	dc2_backwards(60);
	dc3_backwards(60);
	dc1_stop();
	dc4_stop();
//	stop();
}

void back_right(){
	dc1_backwards(60);
	dc4_backwards(60);
	dc2_stop();
	dc3_stop();
//	stop();
}

void right(){
	dc2_backwards(60);
	dc3_backwards(60);
	dc1_forward(60);
	dc4_forward(60);
//	stop();
}

void left(){
	dc1_backwards(60);
	dc4_backwards(60);
	dc2_forward(60);
	dc3_forward(60);
//	stop();
}

void rotate_left(){
	dc1_backwards(60);
	dc3_backwards(60);
	dc2_forward(60);
	dc4_forward(60);
//	stop();
}

void rotate_right(){
	dc2_backwards(60);
	dc4_backwards(60);
	dc1_forward(60);
	dc3_forward(60);
//	stop();
}

//void line(){
//	if(HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0){
//		forward();
//	}
//	else if(HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0){
//		rotate_left();
//	}
//	else if(HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0){
//		rotate_right();
//	}
//	else{
//		stop();
//	}
//}

uint8_t check_line(){

	return sensor_buffer == 0b111 || sensor_buffer == 0b101 || sensor_buffer == 0b110 || sensor_buffer == 0b011;
}

//void line_set(){
//	switch(status) {
//		case FORWARD_1:
//			forward();
//			if(check_line()){
//				status = READYLEFT;
//				stop();
//				count = 20;
//			}
//			break;
//		case READYLEFT:
//			count--;
//			if(count <= 0){
//				count = 20;
//				if(check_line() != 0)
//					forward();
//				else{
//					stop();
//					status = TURNLEFT;
//				}
//			}
//			break;
//		case TURNLEFT:
//			left();
//			if(sensor_buffer == 0b100){
//				status = READYFORWARD;
//				count = 20;
//			}
//			break;
//		case READYFORWARD:
//			left();
//			if(sensor_buffer == 0b010){
//				stop();
//				status = FORWARD_2;
//			}
//			break;
//		case FORWARD_2:
//			forward();
//			if(check_line()){
//				status = READYSTEP2;
//			}
//			break;
//		case READYSTEP2:
//			if(check_line() != 0)
//				forward();
//			else{
//				status = FORWARD_3;
//			}
//			break;
//		case FORWARD_3:
//			forward();
//			if(check_line()){
//				status = STOP;
//			}
//			break;
//		case STOP:
//			stop();
//			break;
//		default:
//			status = FORWARD_1;
//			break;
//	}
//
//}

uint8_t check_status(){
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
}

void sensorCalib(){
	for(int i = 0; i < 3; i++){
		sensor_calib[i] = sensor_value[i];
	}
}

void line_set_temp(){
	switch(status) {
		case CALIB:
			if(button_count == 100){
				sensorCalib();
				status = START;
			}
			break;
		case START:
			if(button_count == 1){
				index = 0;
				status = check_status();
			}
			break;
		case FORWARD:
			forward();
			if(check_line()){
				status = ENDWARDS;
				stop();
				count = 20;
			}
			break;
		case READYBACKWARDS:
			backwards();
			if(check_line()){
				status = READYBACKWARDS2;
			}
			break;
		case READYBACKWARDS2:
			backwards();
			if(check_line() == 0){
				status = BACKWARD;
				stop();
			}
			break;
		case BACKWARD:
			backwards();
			if(check_line()){
				status = ENDBACK;
				stop();
				count = 20;
			}
			break;
		case ENDBACK:
			count--;
			if(count <= 0){
				if(check_line()){
					status = ENDWARDS;
					stop();
					count = 20;
				}
				if(check_line() == 0){
					status = FORWARD;
				}
			}
			break;
		case ENDWARDS:
			count--;
			if(count <= 0){
				count = 20;
				if(check_line() != 0)
					forward();
				else{
					stop();
					status = check_status();
				}
			}
			break;
		case TURNLEFT:
			left();
			if(sensor_buffer == 0b100){
				status = ENDLEFT;
			}
			break;
		case ENDLEFT:
			left();
			if(sensor_buffer == 0b010){
				stop();
				status = check_status();
			}
			break;
		case TURNRIGHT:
			right();
			if(sensor_buffer == 0b001){
				status = ENDRIGHT;
			}
			break;
		case ENDRIGHT:
			right();
			if(sensor_buffer == 0b010){
				stop();
				status = check_status();
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
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setTimer(1,10);
  while (1)
  {
//	  forward();
//	  if(check_line()){
//		  stop();
//		  HAL_Delay(600);
//		  left();
//		  if(HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin)== 0){
//			  stop();
//			  HAL_Delay(600);
//			  forward();
//			  if(check_line()){
//				  step++;
//				  if(step >= 2){
//					  step = 0;
//					  stop();
//				  }
//			  }
//		  }
//	  }

	  if(timer_flag[1]){
		  setTimer(1,10);
//		  forward();
//		  turn_left();
//		  line();
		  button_scan();
		  sensor_scan();
		  line_set_temp();
		  led_blink();
//		  left();
//		  dc3_forward(50);
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
  hadc1.Init.NbrOfConversion = 3;
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
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 144-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /*Configure GPIO pins : CLP_Pin Near_Pin */
  GPIO_InitStruct.Pin = CLP_Pin|Near_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef * htim ){
	timerRun(1);
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
