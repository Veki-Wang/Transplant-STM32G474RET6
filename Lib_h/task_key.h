#ifndef _TASK_KEY_H_
#define _TASK_KEY_H_

#include "main.h"
#include "stm32g4xx_hal.h"
//#include "task_oled_and_init.h"

#define KEY1_Pin            GPIO_PIN_12
#define KEY1_GPIO_Port      GPIOB

#define KEY2_Pin            GPIO_PIN_13
#define KEY2_GPIO_Port      GPIOB

#define KEY3_Pin            GPIO_PIN_15
#define KEY3_GPIO_Port      GPIOB

unsigned char Key_GetCode(void);
unsigned char Key_Get(void);
void Key_LoopDetect(void);
// void key_task(void);

#endif // !