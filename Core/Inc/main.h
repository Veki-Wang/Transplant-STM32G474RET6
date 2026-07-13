/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_oled_and_init.h"
#include "task_receive_t_data.h"
#include "task_key.h"
#include "task_pid.h"
#include "QD4310.h"
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
typedef struct
{
  float initial_yaw;//初始yaw值和第一帧标志
  uint8_t first_frame_received;
  uint8_t data_valid;
  float flag_yaw;
  uint8_t stop_scan;
  uint8_t pid_change;
}Start_FlagStatus;

extern Start_FlagStatus flag;

typedef struct
{
    uint8_t start_flag;
    uint8_t question_num;
    uint8_t scan;
    uint8_t scan_speed;
}scan_config;

extern scan_config scan_init;

extern float now_yaw;
extern uint8_t rx_yaw;
extern uint8_t car_rx_byte;
extern QD4310_t Motor_0;
extern QD4310_t Motor_1;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define key1_Pin GPIO_PIN_12
#define key1_GPIO_Port GPIOB
#define key2_Pin GPIO_PIN_13
#define key2_GPIO_Port GPIOB
#define laser_Pin GPIO_PIN_14
#define laser_GPIO_Port GPIOB
#define key3_Pin GPIO_PIN_15
#define key3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
