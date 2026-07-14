#ifndef TASK_RECEIVE_T_DATA_H
#define TASK_RECEIVE_T_DATA_H



#define LASER_ON() HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET)
#define LASER_OFF() HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET)



void Vision_Task_Init(void);              // 初始化（传入题目号）
void Vision_Task(void);                  // 主循环任务（main只调用这个）


#endif 

