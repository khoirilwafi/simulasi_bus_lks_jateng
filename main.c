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
#include <stdbool.h>
#include "lcd.h"
#include "shift_register.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Lcd_PortType lcd_port[] = {
	LCD_D4_GPIO_Port, LCD_D5_GPIO_Port, LCD_D6_GPIO_Port, LCD_D7_GPIO_Port
};

Lcd_PinType lcd_pin[] = {
	LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

// ----- static data -----

// nama jurusan
static char* jurusan[2] = {
	"UNNES", "UNDIP"
};

// nama halte
static char* halte[9] = {
	"RSND",
	"Rusunawa",
	"Polines",
	"Kesatrian",
	"Kagok",
	"Elizabet",
	"Park UNNES",
	"BNI UNNES",
	"UNNES",
};

// nama status
static char* status[2] = {
	"On The Way",
	"In Transit"
};

// data check poin berisi status dan halte selanjutnya untuk setiap check point
static const uint8_t check_point[2][16][2] = {
	{{1,8},{1,7},{0,6},{0,6},{1,6},{0,5},{1,5},{0,4},{1,4},{0,3},{1,3},{1,2},{0,1},{1,1},{0,0},{1,0}},
	{{1,0},{0,1},{1,1},{0,2},{1,2},{1,3},{0,4},{1,4},{0,5},{1,5},{0,6},{1,6},{0,7},{0,7},{1,7},{1,8}}
};


// ----- public variables -----

// peripheral data holder
bool latch_sw1 = false;
bool latch_sw2 = false;
bool latch_sw3 = false;
bool latch_sw4 = false;
bool latch_sw5 = false;

bool is_waiting = false;
bool is_editing = false;
bool bus_is_here = false;

bool screen_refresh = true;

// index menu
uint8_t page_now = 0;
uint8_t cursor_index = 0;
uint8_t jurusan_index = 0, halte_index = 0;

// bus parameter
uint8_t traffic_speed = 0;
uint8_t bus_a_pos = 0, bus_b_pos = 0;

// task tick holder
uint32_t btn_sw5_last_tick = 0;
uint32_t bus_a_last_tick = 0, bus_b_last_tick = 0;
uint32_t waiting_blink_last_tick = 0;
uint32_t bus_is_here_last_tick = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

// ----- helper function -----
void set_bus_position(uint16_t busA, uint16_t busB);
uint16_t read_adc(void);
int map(int value, int minSrc, int maxSrc, int minDst, int maxDst);

// ----- task -----
void button_task(void);
void screen1_task(void);
void screen2_task(void);
void screen3_task(void);
void bus_check_point_task(uint16_t speed);
uint8_t speed_check_task(void);

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
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);

  Lcd_create(lcd_port, lcd_pin, LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_EN_GPIO_Port, LCD_EN_Pin, LCD_4_BIT_MODE);
  Register_create(CLK_GPIO_Port, CLK_Pin, DATA_GPIO_Port, DATA_Pin, LATCH_GPIO_Port, LATCH_Pin);

  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

  if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_RESET)
  {
	  Lcd_cursor(0, 0);
	  Lcd_string("[TEST PROGRAM]    V1");
	  Lcd_cursor(2, 0);
	  Lcd_string("UJI COBA LKS JATENG ");
	  Lcd_cursor(3, 0);
	  Lcd_string("DEMAK 2023");

	  while(1);
  }

  Lcd_cursor(0, 3);
  Lcd_string("SELAMAT DATANG");
  Lcd_cursor(2, 4);
  Lcd_string("TRANS JATENG");
  Lcd_cursor(3, 1);
  Lcd_string("SIAP MELAYANI ANDA");

  set_bus_position(0, 0);

  HAL_Delay(3000);
  Lcd_clear();

  // initial data value
  bus_a_last_tick = HAL_GetTick();
  bus_b_last_tick = HAL_GetTick();
  waiting_blink_last_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // refresh screen if data change
	  if(screen_refresh == true)
	  {
		  Lcd_clear();

		  switch (page_now)
		  {
		  	  case 0 :
		  		  screen1_task();
			  break;

		  	  case 1 :
		  		  screen2_task();
			  break;

		  	  case 2 :
		  		  screen3_task();
		  	  break;
		  }

		  screen_refresh = false;
	  }


	  // task for blink waiting status
	  if(is_waiting == true)
	  {
		  HAL_GPIO_WritePin(LED_33_RED_GPIO_Port, LED_33_RED_Pin, GPIO_PIN_SET);

		  if(HAL_GetTick() - waiting_blink_last_tick >= 500)
		  {
			  HAL_GPIO_TogglePin(LED_33_GREEN_GPIO_Port, LED_33_GREEN_Pin);
			  waiting_blink_last_tick = HAL_GetTick();
		  }
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LED_33_RED_GPIO_Port, LED_33_RED_Pin, GPIO_PIN_RESET);
	  }


	  // notification bus is here
	  if(bus_is_here == true && (HAL_GetTick() - bus_is_here_last_tick) >= 1500)
	  {
		  HAL_GPIO_WritePin(LED_33_GREEN_GPIO_Port, LED_33_GREEN_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED_33_RED_GPIO_Port, LED_33_RED_Pin, GPIO_PIN_RESET);

		  // matikan buzzer
		  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

		  bus_is_here = false;
		  screen_refresh = true;
	  }
	  else if(bus_is_here == true)
	  {
		  HAL_GPIO_WritePin(LED_33_GREEN_GPIO_Port, LED_33_GREEN_Pin, GPIO_PIN_RESET);

		  // nyalakan buzzer
		  // HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

		  Lcd_cursor(1, 6);
		  Lcd_string("SILAHKAN");
		  Lcd_cursor(2, 4);
		  Lcd_string("MEMASUKI BUS");

		  is_waiting = false;
	  }


	  // task for read button
	  button_task();


	  // task for check speed of bus
	  traffic_speed = speed_check_task();

	  // task for bus chaeck point
	  bus_check_point_task(traffic_speed);


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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_D4_Pin|LCD_D5_Pin|LATCH_Pin|DATA_Pin
                          |CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_D6_Pin|BUZZER_Pin|LCD_BL_Pin|LCD_EN_Pin
                          |LCD_RW_Pin|LCD_RS_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_33_GREEN_Pin|LED_33_RED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin
                           SW5_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin
                          |SW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D5_Pin LATCH_Pin DATA_Pin
                           CLK_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LATCH_Pin|DATA_Pin
                          |CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D6_Pin LCD_EN_Pin LCD_RS_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_D6_Pin|LCD_EN_Pin|LCD_RS_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin LED_33_GREEN_Pin LED_33_RED_Pin LCD_BL_Pin
                           LCD_RW_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|LED_33_GREEN_Pin|LED_33_RED_Pin|LCD_BL_Pin
                          |LCD_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void set_bus_position(uint16_t busA, uint16_t busB)
{
	uint8_t buffer[4];

	buffer[0] = (busB >= 8) ? ~ (0x01 << (busB - 8)) : 0xff;
	buffer[1] = ~ (0x01 << busB);

	buffer[2] = (busA >= 8) ? ~ (0x01 << (busA - 8)) : 0xff;
	buffer[3] = ~ (0x01 << busA);

	Register_send(buffer, 4);
}

uint16_t read_adc(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	uint16_t data_adc = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return data_adc;
}

int map(int value, int minSrc, int maxSrc, int minDst, int maxDst)
{
    int rangeSrc = maxSrc - minSrc;
    int rangeDst = maxDst - minDst;
    int scaled = (value - minSrc) * rangeDst / rangeSrc + minDst;

    return scaled;
}

void screen1_task(void)
{
	Lcd_cursor(0, 0);
	Lcd_string("[Koridor VII]");
	Lcd_cursor(0, 15);
	Lcd_string(jurusan[page_now]);
	Lcd_cursor(2, 0);
	Lcd_string("Status:");
	Lcd_cursor(3, 0);
	Lcd_string("Halte :");

	Lcd_cursor(2, 8);
	Lcd_string(status[check_point[1][bus_b_pos][0]]);
	Lcd_cursor(3, 8);
	Lcd_string("           ");
	Lcd_cursor(3, 8);
	Lcd_string(halte[check_point[1][bus_b_pos][1]]);
}

void screen2_task(void)
{
	Lcd_cursor(0, 0);
	Lcd_string("[Koridor VI]");
	Lcd_cursor(0, 15);
	Lcd_string(jurusan[page_now]);
	Lcd_cursor(2, 0);
	Lcd_string("Status:");
	Lcd_cursor(3, 0);
	Lcd_string("Halte :");

	Lcd_cursor(2, 8);
	Lcd_string(status[check_point[0][bus_a_pos][0]]);
	Lcd_cursor(3, 8);
	Lcd_string("           ");
	Lcd_cursor(3, 8);
	Lcd_string(halte[check_point[0][bus_a_pos][1]]);
}

void screen3_task(void)
{
	Lcd_cursor(0, 0);
	Lcd_string("[TUNGGU BUS]");

	Lcd_cursor(2, 0);
	Lcd_string("Bus Ke:");
	Lcd_cursor(3, 0);
	Lcd_string("Halte :");

	if(is_editing == false)
	{
	  Lcd_cursor(2, 8);
	  Lcd_string(jurusan[jurusan_index]);
	  Lcd_cursor(3, 8);
	  Lcd_string(halte[halte_index]);

	  if(is_waiting == false)
	  {
		  Lcd_cursor(cursor_index + 2, 19);
		  Lcd_string("<");
	  }
	  else
	  {
		  Lcd_cursor(0, 14);
		  Lcd_string("TUNGGU");
	  }
	}
	else
	{
	  if(cursor_index == 0)
	  {
		  Lcd_cursor(2, 8);
		  Lcd_string("[");
		  Lcd_string(jurusan[jurusan_index]);
		  Lcd_cursor(2, 9 + strlen(jurusan[jurusan_index]));
		  Lcd_string("]");

		  Lcd_cursor(3, 8);
		  Lcd_string(halte[halte_index]);
	  }
	  else
	  {
		  Lcd_cursor(3, 8);
		  Lcd_string("[");
		  Lcd_string(halte[halte_index]);
		  Lcd_cursor(3, 9 + strlen(halte[halte_index]));
		  Lcd_string("]");

		  Lcd_cursor(2, 8);
		  Lcd_string(jurusan[jurusan_index]);
	  }
	}
}

void button_task(void)
{
	// check current pins condition
	if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET && latch_sw1 == false)
	{
		if(page_now <= 0)
		{
			page_now = 0;
		}
		else
		{
			page_now -= 1;
			screen_refresh = true;
		}

		latch_sw1 = true;
	}
	else if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET && latch_sw2 == false)
	{
		if(page_now >= 2)
		{
			page_now = 2;
		}
		else
		{
			page_now += 1;
			screen_refresh = true;
		}

		latch_sw2 = true;
	}
	else if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET && latch_sw3 == false)
	{
		if(is_editing)
		{
			if(cursor_index == 0)
			{
				jurusan_index = (jurusan_index >= 1) ? 1 : jurusan_index + 1;
			}
			else
			{
				halte_index = (halte_index >= 8) ? 8 : halte_index + 1;
			}
		}
		else
		{
			cursor_index = 0;
		}

		screen_refresh = true;
		latch_sw3 = true;
	}
	else if(HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == GPIO_PIN_RESET && latch_sw4 == false)
	{
		if(is_editing)
		{
			if(cursor_index == 0)
			{
				jurusan_index = (jurusan_index <= 0) ? 0 : jurusan_index - 1;
			}
			else
			{
				halte_index = (halte_index <= 0) ? 0 : halte_index - 1;
			}
		}
		else
		{
			cursor_index = 1;
		}

		screen_refresh = true;
		latch_sw4 = true;
	}
	else if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_RESET && latch_sw5 == false)
	{
		if(is_waiting == false)
		{
			is_editing = (is_editing) ? false : true;

			btn_sw5_last_tick = HAL_GetTick();
			screen_refresh = true;
		}

		latch_sw5 = true;
	}

	// reset pins condition on release
	if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_SET && latch_sw1 == true)
	{
		latch_sw1 = false;
	}
	else if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_SET && latch_sw2 == true)
	{
		latch_sw2 = false;
	}
	else if(HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_SET && latch_sw3 == true)
	{
		latch_sw3 = false;
	}
	else if(HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin) == GPIO_PIN_SET && latch_sw4 == true)
	{
		latch_sw4 = false;
	}
	else if(HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin) == GPIO_PIN_SET && latch_sw5 == true)
	{
		latch_sw5 = false;
	}

	if(latch_sw5 == true && is_waiting == false)
	{
		if(HAL_GetTick() - btn_sw5_last_tick >= 1000)
		{
			is_waiting = true;
			is_editing = false;

			screen_refresh = true;
		}
	}
}

void bus_check_point_task(uint16_t speed)
{
	// increment check point for bus B
	if(HAL_GetTick() - bus_b_last_tick >= (100 / speed) * 1000)
	{
		bus_b_pos = (bus_b_pos >= 15) ? 0 : bus_b_pos + 1;
		set_bus_position(bus_a_pos, bus_b_pos);

		if(page_now == 0)
		{
			Lcd_cursor(2, 8);
			Lcd_string(status[check_point[1][bus_b_pos][0]]);
			Lcd_cursor(3, 8);
			Lcd_string("           ");
			Lcd_cursor(3, 8);
			Lcd_string(halte[check_point[1][bus_b_pos][1]]);
		}

		// check waiting list
		if(is_waiting == true && jurusan_index == 0 && check_point[1][bus_b_pos][0] == 1 && check_point[1][bus_b_pos][1] == halte_index)
		{
			bus_is_here_last_tick = HAL_GetTick();
			Lcd_clear();
			bus_is_here = true;
		}

		bus_b_last_tick = HAL_GetTick();
	}

	// increment check point for bus A -25kmph from bus B
	if(HAL_GetTick() - bus_a_last_tick >= (125 / (float) speed) * 1000)
	{
		bus_a_pos = (bus_a_pos >= 15) ? 0 : bus_a_pos + 1;
		set_bus_position(bus_a_pos, bus_b_pos);

		if(page_now == 1)
		{
			Lcd_cursor(2, 8);
			Lcd_string(status[check_point[0][bus_a_pos][0]]);
			Lcd_cursor(3, 8);
			Lcd_string("           ");
			Lcd_cursor(3, 8);
			Lcd_string(halte[check_point[0][bus_a_pos][1]]);
		}

		// check waiting list
		if(is_waiting == true && jurusan_index == 1 && check_point[0][bus_a_pos][0] == 1 && check_point[0][bus_a_pos][1] == halte_index)
		{
			bus_is_here_last_tick = HAL_GetTick();
			Lcd_clear();
			bus_is_here = true;
		}

		bus_a_last_tick = HAL_GetTick();
	}
}

uint8_t speed_check_task(void)
{
	uint16_t data_adc = read_adc();
	return map(data_adc, 0, 4095, 1, 100);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM4)
	{
		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
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

