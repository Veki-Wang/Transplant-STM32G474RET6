#ifndef TASK_RECEIVE_T_DATA_H
#define TASK_RECEIVE_T_DATA_H

#include "main.h"
#include <math.h>
#include "huart3_base.h"
#include "task_oled_and_init.h"
#include "task_scan.h"

#define LASER_ON() HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_SET)
#define LASER_OFF() HAL_GPIO_WritePin(laser_GPIO_Port, laser_Pin, GPIO_PIN_RESET)

/* 共用体用来接收浮点数 */
typedef union UnionFloat
{
    uint8_t Array[4]; // 字节数组形式，用于逐字节接收
    float FloatNum;   // 浮点数形式，用于直接读取解析后的坐标
} UnionFloat_t;

/* 接收数据到的阈值的数据*/
typedef  struct
{
    /* data */
    float pos_x;
    float pos_y;
    float yaw;
    float laser_dis_x;
    float laser_dis_y;
    int16_t car_speed;
    uint8_t  car_mode;
} Receive_data;

extern Receive_data vision_data;      // 视觉数据的存储


/*通信的状态机发送数据的帧头和帧尾*/
typedef struct {
    const uint8_t START[3];    // 启动帧: A5 FF 5A
    const uint8_t DATA_NUM;     // 应答帧: B6 01 6B
    const uint8_t CMD_HEAD;   // 帧头 A5
    const uint8_t CMD_TAIL;   // 帧尾 5A
    const uint8_t DATA1_HEAD; // 第一组数据头 B6
    const uint8_t DATA1_TAIL; // 第一组数据尾 6B
    const uint8_t CMD_OK;     // 完成指令 66
    const uint8_t CAR_DATA2_HEAD; // 第二组数据头 C7
    const uint8_t CAR_DATA2_TAIL; // 第二组数据尾 7C
    const uint8_t VF_TIMMEOUT; // 接收超时时间，单位ms
} Vision_Frame_t;

extern Vision_Frame_t Frame;

/*通信的状态机*/
typedef enum
{
    VISION_IDLE,
    VISION_ACK,
    VISION_WAIT_CONDITION,
    VISION_SEND_OK,
    VISION_RECV_DATA2,
    VISION_FINISH
} Vision_State_t;

extern Vision_State_t vision_state;  

void Vision_Task_Init(void);              // 初始化（传入题目号）
void Vision_Task(void);                  // 主循环任务（main只调用这个）


#endif 

