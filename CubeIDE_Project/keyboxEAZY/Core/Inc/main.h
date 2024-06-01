/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void GPIO_Poll (uint16_t * p_button_state);
void DPAD_Poll (uint8_t * p_dpad_state);
void DrawAll(uint8_t* button_state,uint8_t* dpad_state);
void Draw_Buttons(uint8_t* button_state);
void Draw_DPAD(uint8_t* dpad_state);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEFT_Pin GPIO_PIN_0
#define LEFT_GPIO_Port GPIOC
#define DOWN_Pin GPIO_PIN_1
#define DOWN_GPIO_Port GPIOC
#define RIGHT_Pin GPIO_PIN_2
#define RIGHT_GPIO_Port GPIOC
#define UP_Pin GPIO_PIN_3
#define UP_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA
#define LK_Pin GPIO_PIN_4
#define LK_GPIO_Port GPIOA
#define LP_Pin GPIO_PIN_5
#define LP_GPIO_Port GPIOA
#define MK_Pin GPIO_PIN_6
#define MK_GPIO_Port GPIOA
#define MP_Pin GPIO_PIN_7
#define MP_GPIO_Port GPIOA
#define HK_Pin GPIO_PIN_4
#define HK_GPIO_Port GPIOC
#define HP_Pin GPIO_PIN_5
#define HP_GPIO_Port GPIOC
#define EXK_Pin GPIO_PIN_0
#define EXK_GPIO_Port GPIOB
#define EXP_Pin GPIO_PIN_1
#define EXP_GPIO_Port GPIOB
#define OPT6_Pin GPIO_PIN_11
#define OPT6_GPIO_Port GPIOC
#define OPT5_Pin GPIO_PIN_12
#define OPT5_GPIO_Port GPIOC
#define OPT4_Pin GPIO_PIN_5
#define OPT4_GPIO_Port GPIOB
#define OPT3_Pin GPIO_PIN_6
#define OPT3_GPIO_Port GPIOB
#define OPT2_Pin GPIO_PIN_7
#define OPT2_GPIO_Port GPIOB
#define OPT1_Pin GPIO_PIN_8
#define OPT1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define THROTTLE_ENABLED				0					// 1 for enabled, 0 for disabled
#define RUDDER_ENABLED					0					// 1 for enabled, 0 for disabled

#define JOYSTICK_AXIS_NUM				0
#define SECONDARY_AXIS_NUM				0

#define BUTTONS_NUM						14
#define DPAD_ENABLED					1					// 1 for enabled, 0 for disabled
#define LEDS_NUM						0

#define FILTER_WINDOW_SIZE				10

#define ADC_CHANNELS_NUM				8


#define AXIS_NUM						((THROTTLE_ENABLED) + (JOYSTICK_AXIS_NUM) \
										+ (SECONDARY_AXIS_NUM) + (RUDDER_ENABLED))

#if (JOYSTICK_AXIS_NUM > 0)
	#define JOYSTICK_AXIS_ENABLED  		1
#else
	#define JOYSTICK_AXIS_ENABLED  		0
#endif

#if (SECONDARY_AXIS_NUM > 0)
	#define SECONDARY_AXIS_ENABLED  	1
#else
	#define SECONDARY_AXIS_ENABLED  	0
#endif

#if (BUTTONS_NUM > 0)
	#define BUTTONS_ENABLED  			1
#else
	#define BUTTONS_ENABLED  			0
#endif

#define DEVICE_ID1					0x1FFFF7E8
#define DEVICE_ID2					0x1FFFF7EA
#define DEVICE_ID3					0x1FFFF7EC
#define DEVICE_ID4					0x1FFFF7F0
#define DPAD_UP            0x00
#define DPAD_UPRIGHT       0x01
#define DPAD_RIGHT         0x02
#define DPAD_DOWNRIGHT     0x03
#define DPAD_DOWN          0x04
#define DPAD_DOWNLEFT      0x05
#define DPAD_LEFT          0x06
#define DPAD_UPLEFT        0x07
#define DPAD_NONE          0x0F

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
