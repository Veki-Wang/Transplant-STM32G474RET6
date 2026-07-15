#ifndef TASK_RECEIVE_T_DATA_H
#define TASK_RECEIVE_T_DATA_H



#define LASER_ON() HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET)
#define LASER_OFF() HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET)



void Vision_Task_Init(void);              // 初始化（传入题目号）
void Vision_Task(void);                  // 主循环任务（main只调用这个）

/* ===================== 云台通讯 (UART5 ↔ 菜单) ===================== */
void GimbalComm_Init(void);              // 初始化 UART5 接收中断
void GimbalComm_Send(void);              // 发送当前 PID/前馈值到菜单 (E9...9E)
void GimbalComm_FeedByte(uint8_t ch);    // 接收状态机, 由 HAL 回调喂入每个字节
void GimbalComm_ApplyRecvData(void);     // 将收到的数据应用到 PID 和 kf_table


#endif 

