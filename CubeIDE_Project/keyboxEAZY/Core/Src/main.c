/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REPORT_SIZ 2*AXIS_NUM+2*BUTTONS_ENABLED+DPAD_ENABLED
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

//volatile uint8_t data_ready = 0;

#if (AXIS_NUM > 0)
uint32_t filter_buf[AXIS_NUM][FILTER_WINDOW_SIZE];
#endif
uint8_t report_data[REPORT_SIZ];

uint8_t buttons_state[2];
uint8_t dpad_state;
extern const SSD1306_Font_t Font_7x10;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
uint16_t FilterWindow (uint32_t * p_buf, uint16_t new_val);

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
  MX_I2C3_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
  //ssd1306_Fill(White);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Hello World!", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(600);
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  			GPIO_Poll((uint16_t*)&buttons_state);
	  			DPAD_Poll(&dpad_state);
#if (AXIS_NUM > 0)
	  			for (uint8_t i=0; i<AXIS_NUM; i++)
	  				report_data[i] = FilterWindow(filter_buf[i], ADC_data[i]);
#endif

#if (BUTTONS_ENABLED == 1)
				report_data[2 * AXIS_NUM] = buttons_state[0];
				report_data[2 * AXIS_NUM + 1] = buttons_state[1];
#endif
#if (DPAD_ENABLED == 1)
				report_data[2 * AXIS_NUM + 2 * BUTTONS_ENABLED] = dpad_state;

#endif
	  			uint8_t errrr = USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &report_data, (2 * AXIS_NUM + 2 * BUTTONS_ENABLED + 1)*sizeof(uint8_t));
	  			if (errrr == 3) {
	  				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0); //LED licht op wanneer de USB lib een error returnt (als de polling niet kan bijhouden)
	  				HAL_Delay(500);
				} else {
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
				}

	  			DrawAll(buttons_state, &dpad_state);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 800000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEFT_Pin DOWN_Pin RIGHT_Pin UP_Pin
                           HK_Pin HP_Pin OPT6_Pin OPT5_Pin */
  GPIO_InitStruct.Pin = LEFT_Pin|DOWN_Pin|RIGHT_Pin|UP_Pin
                          |HK_Pin|HP_Pin|OPT6_Pin|OPT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LK_Pin LP_Pin MK_Pin MP_Pin */
  GPIO_InitStruct.Pin = LK_Pin|LP_Pin|MK_Pin|MP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EXK_Pin EXP_Pin OPT4_Pin OPT3_Pin
                           OPT2_Pin OPT1_Pin */
  GPIO_InitStruct.Pin = EXK_Pin|EXP_Pin|OPT4_Pin|OPT3_Pin
                          |OPT2_Pin|OPT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void GPIO_Poll (uint16_t * p_button_state)
{
	*p_button_state = 0;
	*p_button_state |= HAL_GPIO_ReadPin(LP_GPIO_Port, LP_Pin);
	*p_button_state |= HAL_GPIO_ReadPin(MP_GPIO_Port, MP_Pin) << 1;
	*p_button_state |= HAL_GPIO_ReadPin(HP_GPIO_Port, HP_Pin) << 2;
	*p_button_state |= HAL_GPIO_ReadPin(EXP_GPIO_Port, EXP_Pin) << 3;
	*p_button_state |= HAL_GPIO_ReadPin(LK_GPIO_Port, LK_Pin) << 4;
	*p_button_state |= HAL_GPIO_ReadPin(MK_GPIO_Port, MK_Pin) << 5;
	*p_button_state |= HAL_GPIO_ReadPin(HK_GPIO_Port, HK_Pin) << 6;
	*p_button_state |= HAL_GPIO_ReadPin(EXK_GPIO_Port, EXK_Pin) << 7;
	*p_button_state |= HAL_GPIO_ReadPin(OPT1_GPIO_Port, OPT1_Pin) << 8;
	*p_button_state |= HAL_GPIO_ReadPin(OPT2_GPIO_Port, OPT2_Pin) << 9;
	*p_button_state |= HAL_GPIO_ReadPin(OPT3_GPIO_Port, OPT3_Pin) << 10;
	*p_button_state |= HAL_GPIO_ReadPin(OPT4_GPIO_Port, OPT4_Pin) << 11;
	*p_button_state |= HAL_GPIO_ReadPin(OPT5_GPIO_Port, OPT5_Pin) << 12;
	*p_button_state |= HAL_GPIO_ReadPin(OPT6_GPIO_Port, OPT6_Pin) << 13;
	*p_button_state ^= 0x3FFF;
	return;
}
void DPAD_Poll (uint8_t * p_dpad_state)
{
	*p_dpad_state 	= DPAD_NONE;
	uint8_t dpad_LR	= DPAD_NONE;
	uint8_t dpad_UD	= DPAD_NONE;
	uint8_t up 		= !(HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin));
	uint8_t right 	= !(HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin));
	uint8_t down 	= !(HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin));
	uint8_t left 	= !(HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin));

	//SOCD Cleaning: UP + DOWN moet UP worden, LEFT + RIGHT moet neutral worden

	if (down && (!up)) {
		dpad_UD	= DPAD_DOWN;
	}

	if (up) {
		dpad_UD	= DPAD_UP;
	}

	if (left && right) {
		dpad_LR = DPAD_NONE;
	} else if (left && (!right)) {
		dpad_LR = DPAD_LEFT;
	} else if ((!left) && right) {
		dpad_LR = DPAD_RIGHT;
	}

	if (dpad_LR != DPAD_NONE) {
		if (dpad_UD == DPAD_NONE) {
			*p_dpad_state = dpad_LR;	//cardinal LR
		} else {						//Non-cardinal
			if(dpad_UD == DPAD_UP) {
				if (dpad_LR == DPAD_RIGHT) {
					*p_dpad_state = DPAD_UPRIGHT;
				} else { //must be left
					*p_dpad_state = DPAD_UPLEFT;
				}
			} else { //must be down
				if (dpad_LR == DPAD_RIGHT) {
					*p_dpad_state = DPAD_DOWNRIGHT;
				} else { //must be left
					*p_dpad_state = DPAD_DOWNLEFT;
				}
			}
		}
	} else if (dpad_UD != DPAD_NONE) {
		*p_dpad_state = dpad_UD;		//cardinal UD
	}

	return;
}
void DrawAll(uint8_t* button_state,uint8_t* dpad_state) {
	ssd1306_Fill(Black);
	Draw_Buttons(button_state);
	Draw_DPAD(dpad_state);
	ssd1306_UpdateScreen();
	return;
}

void Draw_Buttons(uint8_t* button_state) {
	((*button_state & (0x1 << 0)) ? ssd1306_FillCircle : ssd1306_DrawCircle)(66, 16, 7, White); //LP
	((*button_state & (0x1 << 1)) ? ssd1306_FillCircle : ssd1306_DrawCircle)(82, 10, 7, White); //MP
	((*button_state & (0x1 << 2)) ? ssd1306_FillCircle : ssd1306_DrawCircle)(98, 10, 7, White); //HP
	((*button_state & (0x1 << 3)) ? ssd1306_FillCircle : ssd1306_DrawCircle)(114, 10, 7, White); //EXP
	((*button_state & (0x1 << 4)) ? ssd1306_FillCircle : ssd1306_DrawCircle)(63, 33, 7, White); //LK
	((*button_state & (0x1 << 5)) ? ssd1306_FillCircle : ssd1306_DrawCircle)(79, 28, 7, White); //MK
	((*button_state & (0x1 << 6)) ? ssd1306_FillCircle : ssd1306_DrawCircle)(95, 28, 7, White); //HK
	((*button_state & (0x1 << 7)) ? ssd1306_FillCircle : ssd1306_DrawCircle)(111, 28, 7, White); //EXK

//knoppen worden als volle of holle cirkels afgdrukt op basis van hun status (ingedrukt of niet)
	return;
}
void Draw_DPAD(uint8_t* dpad_state) {
	ssd1306_DrawCircle(56, 52, 9, White); //knoppen worden als volle of holle cirkels afgdrukt op basis van hun status (ingedrukt of niet) 
	ssd1306_DrawCircle(17, 16, 7, White); //(ook de SOCD cleaning wordt getoond doordat we de dpad_state gebruiken)
	ssd1306_DrawCircle(33, 16, 7, White);
	ssd1306_DrawCircle(48, 22, 7, White);
	switch (*dpad_state) {
		case DPAD_UP:
			ssd1306_FillCircle(56, 52, 9, White);
			break;
		case DPAD_UPRIGHT:
			ssd1306_FillCircle(56, 52, 9, White);
			ssd1306_FillCircle(48, 22, 7, White);
			break;
		case DPAD_RIGHT:
			ssd1306_FillCircle(48, 22, 7, White);
			break;
		case DPAD_DOWNRIGHT:
			ssd1306_FillCircle(33, 16, 7, White);
			ssd1306_FillCircle(48, 22, 7, White);
			break;
		case DPAD_DOWN:
			ssd1306_FillCircle(33, 16, 7, White);
			break;
		case DPAD_DOWNLEFT:
			ssd1306_FillCircle(17, 16, 7, White);
			ssd1306_FillCircle(33, 16, 7, White);
			break;
		case DPAD_LEFT:
			ssd1306_FillCircle(17, 16, 7, White);
			break;
		case DPAD_UPLEFT:
			ssd1306_FillCircle(56, 52, 9, White);
			ssd1306_FillCircle(17, 16, 7, White);
			break;
		default:
			break;
	}
	return;
}

uint16_t FilterWindow (uint32_t * p_buf, uint16_t new_val)
{
	uint16_t ret;

	p_buf[FILTER_WINDOW_SIZE-1] = new_val;

	for (uint8_t i=2; i<=FILTER_WINDOW_SIZE; i++)
	{
		p_buf[FILTER_WINDOW_SIZE-1] += p_buf[FILTER_WINDOW_SIZE - i];
	}

	p_buf[FILTER_WINDOW_SIZE-1] /= FILTER_WINDOW_SIZE;

	ret = p_buf[FILTER_WINDOW_SIZE-1];

	for (int i=0; i<FILTER_WINDOW_SIZE; i++)
	{
		p_buf[i-1] = p_buf[i];
	}

		return ret;
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
