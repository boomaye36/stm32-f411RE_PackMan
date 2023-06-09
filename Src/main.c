/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c-lcd.h"
#include <string.h>
#include <stdlib.h>
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
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef enum{
	ING,
	WIN,
	OVER
}_Game_status;
_Game_status game_status=ING;
typedef enum{
	LEVEL1=1,
	LEVEL2,
	LEVEL3
}_Level;
_Level level=LEVEL1;
typedef enum{
	UP,
	DOWN,
	RIGHT,
	LEFT,
	NONE
}_Direction;
typedef struct{
	int row;
	int col;
	int image_num;
	int past_position[2][16];
	uint8_t prey;
}_Character;

typedef struct{
	int row;
	int col;
	int image_num;
	uint8_t clock_before;
}_Enemy;

uint32_t XY[2];
//char uart_buf[30];

uint8_t ClockFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = 8;
//  RCC_OscInitStruct.PLL.PLLN = 180;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Activate the Over-Drive mode
//  */
//  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

_Direction readJoystick();
void MoveCharacter(_Character *character, _Direction direction);
void DisplayLCD(_Character *character);
void DisplayEnemy(_Enemy enemy);
_Game_status GameCHK(_Character *character, _Enemy *enemy);
void MoveEnemy(_Enemy *enemy, _Character character, uint8_t pulse);
void StartSound();
void LoseSound();
void WinSound();
void LevelupInit(_Character *character, _Enemy *enemy);



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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, XY, 2);	// start ADC in DMA mode for Multi channel


  lcd_init();


  // write to CGRAM
  char pac1[] = {0x07, 0x0F, 0x1E, 0x1C, 0x1C, 0x1E, 0x0F, 0x07};
  char pac2[] = {0x07, 0x0F, 0x1E, 0x1C, 0x1E, 0x1F, 0x0F, 0x00};

  char pac3[] = {0x1C, 0x1E, 0x0F, 0x07, 0x07, 0x0F, 0x1E, 0x1C};
  char pac4[] = {0x1C, 0x1E, 0x0F, 0x07, 0x0F, 0x1F, 0x1E, 0x00};

  char enemy[] = {0x1F, 0x15, 0x1F, 0x1F, 0x0E, 0x0A, 0x0A, 0x1B};

  lcd_send_cmd(0x40);
  for(int i=0; i<8; i++) lcd_send_data(pac1[i]);

  lcd_send_cmd(0x40+8);
  for(int i=0; i<8; i++) lcd_send_data(pac2[i]);

  lcd_send_cmd(0x40+16);
  for (int i=0; i<8; i++) lcd_send_data(pac3[i]);

  lcd_send_cmd(0x40+24);
  for (int i=0; i<8; i++) lcd_send_data(pac4[i]);

  lcd_send_cmd(0x40+32);
  for (int i=0; i<8; i++) lcd_send_data(enemy[i]);

  lcd_put_cur(0, 0);
  lcd_send_string("press the button");

  lcd_put_cur(1, 4);
  lcd_send_string("to start");

  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)){
	  HAL_Delay(100);
  }

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  StartSound();

  lcd_clear();
  lcd_put_cur(0, 1);
  lcd_send_string("LEVEL 1");
  HAL_Delay(500);

  lcd_put_cur(0, 9);
  lcd_send_string("start!");
  HAL_Delay(800);

  // init _Character
  _Character pacman;
  memset(&pacman, 0, sizeof(pacman));
  pacman.prey=31;
  _Enemy octopus;

  // init _Enemy
  memset(&pacman, 0, sizeof(octopus));
  octopus.image_num=4;
  octopus.row=1;
  octopus.col=8;

  lcd_clear();
  lcd_put_cur(pacman.row, pacman.col);
  lcd_send_data(pacman.image_num);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(game_status==ING){
		  MoveCharacter(&pacman, readJoystick());
		  DisplayLCD(&pacman);
		  MoveEnemy(&octopus, pacman, ClockFlag);
		  DisplayEnemy(octopus);

		  game_status=GameCHK(&pacman, &octopus);

		  HAL_Delay(100);
		  TIM3->CCR3 = 0;

	  }else if(game_status==WIN){
		  lcd_put_cur(0,4);
		  lcd_send_string("YOU WIN");
		  lcd_put_cur(1,0);
		  lcd_send_string("Congratulations!");
	  }else if(game_status==OVER){
		  lcd_put_cur(0,4);
		  lcd_send_string("YOU LOSE");
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 9000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
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
  htim3.Init.Prescaler = 230-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USART_TX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(USART_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

_Direction readJoystick(){
	  if(XY[1]>4050&& HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)){
		  return RIGHT;
	  }else if(XY[1]<50&& HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)){
		  return LEFT;
	  }else if ( XY[1] > 4050 ){
		  return DOWN;
	  }
	  else if(XY[1] < 50){
		  return UP;
	  }
	  else{
		  return NONE;
	  }
}

void MoveCharacter(_Character *character, _Direction direction){

	switch(direction){
	case RIGHT:
		character->col++;
		if(character->col > 15) character->col = 15;
		character->image_num &= ~(0x2);
		character->image_num ^= 1;
		break;
	case LEFT:
		character->col--;
		if(character->col < 0) character->col = 0;
		character->image_num |= 0x2;
		character->image_num ^= 1;
		break;
	case UP:
		character->row--;
		if(character->row < 0) character->row = 0;
		break;
	case DOWN:
		character->row++;
		if(character->row > 1) character->row = 1;
		break;
	default :
		break;
	}
	character->past_position[character->row][character->col]=1;

}

void DisplayLCD(_Character *character){
	uint8_t chk=0;
	lcd_clear();
	lcd_put_cur(character->row, character->col);
	lcd_send_data(character->image_num);
	for(int i=0;i<=1;i++){
		for(int j=0;j<16;j++){
			if(character->past_position[i][j]!=1){
				lcd_put_cur(i,j);
				lcd_send_data(0xa5);
				chk++;
			}
		}
	}
	if(chk < character->prey){
		TIM3->CCR3 = TIM3->ARR /2;
		character->prey=chk;
	}
}

void MoveEnemy(_Enemy *enemy, _Character character, uint8_t pulse){
	uint8_t move = rand()%2;

	if(pulse==1 && enemy->clock_before==0){
		if(move==0){
			if(enemy->row!=character.row) enemy->row=character.row;
		}else if(move==1) {
			if(enemy->col > character.col){
				enemy->col--;
				if(enemy->col<0) enemy->col=0;
			}else if(enemy->col < character.col){
				enemy->col++;
				if(enemy->col>15) enemy->col=15;
			}
		}
	}

	enemy->clock_before=pulse;
}

void DisplayEnemy(_Enemy enemy){
	lcd_put_cur(enemy.row, enemy.col);
	lcd_send_data(enemy.image_num);
}

_Game_status GameCHK(_Character *character, _Enemy *enemy){
	uint8_t chk=0;
	for(int i=0;i<=1;i++){
		for(int j=0;j<16;j++){
			if(character->past_position[i][j]==1) chk++;
		}
	}
	if(character->row==enemy->row && character->col==enemy->col){
		LoseSound();
		return OVER;
	}
	else if(chk==32 && level==1){
		WinSound();
		lcd_clear();
		lcd_put_cur(0, 1);
		lcd_send_string("LEVEL 2");
		HAL_Delay(500);
		lcd_put_cur(0, 9);
		lcd_send_string("start!");
		HAL_Delay(800);
		LevelupInit(character, enemy);
		TIM2->PSC = 6750;
		return game_status;
	}else if(chk==32 && level==2){
		WinSound();
		lcd_clear();
		lcd_put_cur(0, 1);
		lcd_send_string("LEVEL 3");
		HAL_Delay(500);
		lcd_put_cur(0, 9);
		lcd_send_string("start!");
		HAL_Delay(800);
		LevelupInit(character, enemy);
		TIM2->PSC = 4500;
		return game_status;
	}else if(chk==32 && level==3){
		WinSound();
		return WIN;
	}
	else return game_status;
}

void LevelupInit(_Character *character, _Enemy *enemy){
	level++;
	character->row=0;
	character->col=0;
	character->prey=31;
	for(int i=0;i<=1;i++){
		for(int j=0;j<16;j++){
			character->past_position[i][j]=0;
		}
	}
	enemy->row=1;
	enemy->col=8;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	ClockFlag ^= 0x01;
}

void StartSound(){
	TIM3->ARR = 156;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(100);
	TIM3->ARR = 111;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(100);
	// setting for prey eating sound
	TIM3->ARR = 1060;
	TIM3->CCR3 = 0;
}

void LoseSound(){
	TIM3->ARR = 290;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(80);
	TIM3->ARR = 391;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(80);
	TIM3->ARR = 290;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(80);
	TIM3->ARR = 391;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(80);
}

void WinSound(){
	TIM3->ARR = 593;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(100);
	TIM3->CCR3 = 0;
	HAL_Delay(10);
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(100);
	TIM3->CCR3 = 0;
	HAL_Delay(10);
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(100);
	TIM3->CCR3 = 0;
	HAL_Delay(10);
	TIM3->ARR = 767;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(100);
	TIM3->ARR = 593;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(300);
	TIM3->ARR = 508;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(300);
	TIM3->ARR = 1029;
	TIM3->CCR3 = TIM3->ARR / 2;
	HAL_Delay(300);
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
