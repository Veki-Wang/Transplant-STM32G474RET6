#ifndef _HUART3_BASE_H//nano2->3
#define _HUART3_BASE_H

#include "main.h"
#include "task_receive_t_data.h"

extern UART_HandleTypeDef huart3;

void USART3_Send_Byte(uint8_t byte);
void Vision_Send_Start(void);//A5 FF 5A
void Vision_Send_OK(void);//A5 66 5A


#endif 
