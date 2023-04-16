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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t rxData;
int speed=90;
int line_follow = 0;

int sensor_read = 0x00000;
int position;

float Kp = 0.002; //set up the constants value
float Ki = 0.001;
float Kd = 15;
//float Kr = 0;
int P, I, D/*, R*/;
int lastError = 0;
int errors[10] = {0,0,0,0,0,0,0,0,0,0};
int error_sum = 0;
int last_end = 0;	// 0 -> Left, 1 -> Right
int last_idle = 0;

const uint8_t maxspeedr = 100;
const uint8_t maxspeedl = 100;
const uint8_t basespeedr = 92;
const uint8_t basespeedl = 92;
const int ARR = 10;

int actives = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void motor_control (double pos_right, double pos_left)
{
	if (pos_left < 0 )
	{
		HAL_GPIO_WritePin(INT3_GPIO_Port, INT3_Pin, 1);
		HAL_GPIO_WritePin(INT4_GPIO_Port, INT4_Pin, 0);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, -ARR*pos_left);
	}
	else if (pos_left > 0)
	{
		HAL_GPIO_WritePin(INT3_GPIO_Port, INT3_Pin, 0);
		HAL_GPIO_WritePin(INT4_GPIO_Port, INT4_Pin, 1);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*pos_left);
	}
	else
	{
		HAL_GPIO_WritePin(INT3_GPIO_Port, INT3_Pin, 0);
		HAL_GPIO_WritePin(INT4_GPIO_Port, INT4_Pin, 0);
	}
	if (pos_right < 0 )
	{
		HAL_GPIO_WritePin(INT1_GPIO_Port, INT1_Pin, 1);
		HAL_GPIO_WritePin(INT2_GPIO_Port, INT2_Pin, 0);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, -ARR*pos_right);
	}
	else if (pos_right > 0)
	{
		HAL_GPIO_WritePin(INT1_GPIO_Port, INT1_Pin, 0);
		HAL_GPIO_WritePin(INT2_GPIO_Port, INT2_Pin, 1);
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*pos_right);
	}
	else
	{
		HAL_GPIO_WritePin(INT1_GPIO_Port, INT1_Pin, 0);
		HAL_GPIO_WritePin(INT2_GPIO_Port, INT2_Pin, 0);
	}
}

/*void sharp_turn () {

	if (last_idle < 25)
	{
		if (last_end == 1)
			motor_control(-20, 100);
		else
			motor_control(100, -20);
	}
	else
	{
		if (last_end == 1)
			motor_control(-53, 70);
		else
			motor_control(70, -53);
	}
}*/

void finish_line()
{
	if (last_idle > 25)
		motor_control(0,0);
}

int QTR8_read ()
{
	//HAL_GPIO_WritePin(LEDON_GPIO_Port, LEDON_Pin, 1);

	Set_Pin_Output(SENSOR1_GPIO_Port, SENSOR1_Pin);
	Set_Pin_Output(SENSOR2_GPIO_Port, SENSOR2_Pin);
	Set_Pin_Output(SENSOR3_GPIO_Port, SENSOR3_Pin);
	Set_Pin_Output(SENSOR4_GPIO_Port, SENSOR4_Pin);
	Set_Pin_Output(SENSOR5_GPIO_Port, SENSOR5_Pin);
	//Set_Pin_Output(SENSOR6_GPIO_Port, SENSOR6_Pin);
	//Set_Pin_Output(SENSOR7_GPIO_Port, SENSOR7_Pin);
	//Set_Pin_Output(SENSOR8_GPIO_Port, SENSOR8_Pin);

	HAL_GPIO_WritePin (SENSOR1_GPIO_Port, SENSOR1_Pin, 1);
	HAL_GPIO_WritePin (SENSOR2_GPIO_Port, SENSOR2_Pin, 1);
	HAL_GPIO_WritePin (SENSOR3_GPIO_Port, SENSOR3_Pin, 1);
	HAL_GPIO_WritePin (SENSOR4_GPIO_Port, SENSOR4_Pin, 1);
	HAL_GPIO_WritePin (SENSOR5_GPIO_Port, SENSOR5_Pin, 1);
	//HAL_GPIO_WritePin (SENSOR6_GPIO_Port, SENSOR6_Pin, 1);
	//HAL_GPIO_WritePin (SENSOR7_GPIO_Port, SENSOR7_Pin, 1);
	//HAL_GPIO_WritePin (SENSOR8_GPIO_Port, SENSOR8_Pin, 1);

	delay_us(12);

	Set_Pin_Input(SENSOR1_GPIO_Port, SENSOR1_Pin);
	Set_Pin_Input(SENSOR2_GPIO_Port, SENSOR2_Pin);
	Set_Pin_Input(SENSOR3_GPIO_Port, SENSOR3_Pin);
	Set_Pin_Input(SENSOR4_GPIO_Port, SENSOR4_Pin);
	Set_Pin_Input(SENSOR5_GPIO_Port, SENSOR5_Pin);
	//Set_Pin_Input(SENSOR6_GPIO_Port, SENSOR6_Pin);
	//Set_Pin_Input(SENSOR7_GPIO_Port, SENSOR7_Pin);
	//Set_Pin_Input(SENSOR8_GPIO_Port, SENSOR8_Pin);

	// Threshold
	delay_us(6000);

	sensor_read = 0x00000;
	int pos = 0;
	int active = 0;

	if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin)) {
		sensor_read |= 0x00001;
		pos += 1000;
    active++;
		last_end = 1;
	}
	if (HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin)) {
		sensor_read |= 0x00010;
		pos += 2000;
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin)) {
		sensor_read |= 0x00100;
		pos += 3000;
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin)) {
		sensor_read |= 0x01000;
		pos += 4000;
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin)) {
		sensor_read |= 0x10000;
		pos += 5000;
    active++;
  /*}
	if (HAL_GPIO_ReadPin(SENSOR6_GPIO_Port, SENSOR6_Pin)) {
		sensor_read |= 0x00100000;
		pos += 6000;
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR7_GPIO_Port, SENSOR7_Pin)) {
		sensor_read |= 0x01000000;
		pos += 7000;
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR8_GPIO_Port, SENSOR8_Pin)) {
		sensor_read |= 0x10000000;
		pos += 8000;
    active++*/
		last_end = 0;
  }

  //HAL_GPIO_WritePin(LEDON_GPIO_Port, LEDON_Pin, 0);

  actives = active;
	position = pos/active;

	if (actives == 0)
		last_idle++;
	else
		last_idle = 0;

	return pos/active;
}

void forward_brake(int pos_right, int pos_left)
{
	if (actives == 0)
		//sharp_turn();
		finish_line();
	else
	  motor_control(pos_right, pos_left);
}

void past_errors (int error)
{
  for (int i = 9; i > 0; i--)
      errors[i] = errors[i-1];
  errors[0] = error;
}

int errors_sum (int index, int abs)
{
  int sum = 0;
  for (int i = 0; i < index; i++)
  {
    if ((abs == 1) & (errors[i] < 0))
      sum += -errors[i];
    else
      sum += errors[i];
  }
  return sum;
}

void PID_control() {
	uint16_t position = QTR8_read();
  int error = 2250 - position;
	past_errors(error);

  P = error;
  I = errors_sum(5, 0);
  D = error - lastError;
  //R = errors_sum(5, 1);
  lastError = error;

  int motorspeed = P*Kp + I*Ki + D*Kd;

  int motorspeedl = basespeedl + motorspeed /*- R*Kr*/;
  int motorspeedr = basespeedr - motorspeed /*- R*Kr*/;

  if (motorspeedl > maxspeedl)
    motorspeedl = maxspeedl;
  if (motorspeedr > maxspeedr)
    motorspeedr = maxspeedr;

	forward_brake(motorspeedr, motorspeedl);
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
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,&rxData,1); // Enabling interrupt receive

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (line_follow==1)
		  PID_control();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65534;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SENSOR5_Pin|SENSOR4_Pin|SENSOR3_Pin|SENSOR2_Pin
                          |SENSOR1_Pin|INT4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INT3_Pin|INT2_Pin|INT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR5_Pin SENSOR4_Pin SENSOR3_Pin SENSOR2_Pin
                           SENSOR1_Pin INT4_Pin */
  GPIO_InitStruct.Pin = SENSOR5_Pin|SENSOR4_Pin|SENSOR3_Pin|SENSOR2_Pin
                          |SENSOR1_Pin|INT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INT3_Pin INT2_Pin INT1_Pin */
  GPIO_InitStruct.Pin = INT3_Pin|INT2_Pin|INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)
  {
	  if (rxData>47 && rxData<57) // '0'-'9'
	  {
		  speed=10*(rxData-48);
		  __HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*speed);
		  __HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*speed);
	  }
	  else if (rxData==113) //'q'
	  {
		  speed=100;
		  __HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, ARR*speed);
		  __HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, ARR*speed);
	  }
	  else if (rxData==70) // 'F'
	  {
		  motor_control(speed, speed);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	  }
	  else if (rxData==66) // 'B'
	  {
		  motor_control(-speed, -speed);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	  }
	  else if (rxData==76) // 'L'
	  {
		  motor_control(speed, -speed);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	  }
	  else if (rxData==82) // 'R'
	  {
		  motor_control(-speed, speed);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	  }
	  else if (rxData==120) // Ascii value of 'x'
	  {
		  line_follow = 0;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);

	  }
	  else if (rxData==88) // Ascii value of 'X'
	  {
	      line_follow = 1;
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	  }
	  else
	  {
		  motor_control(0, 0);
	  }
  HAL_UART_Receive_IT(&huart1,&rxData,1); // Enabling interrupt receive again
  }
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
