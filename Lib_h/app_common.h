/*
 * @file    app_common.h
 * @brief   业务模块公共头文件
 * @author  zhangyong
 * @date    2024-06-10
 */
#ifndef __APP_COMMON_H
#define __APP_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==================================================
   1. 标准库头文件（最底层依赖）
   ================================================== */
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* ==================================================
   2. 底层HAL与外设驱动头文件
   ================================================== */
#include "main.h"
#include "huart3_base.h"

/* ==================================================
   3. 公共类型统一定义区
   所有跨文件共用的结构体、枚举、共用体全部集中在此
   ================================================== */

/* ----- PID控制相关 ----- */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kf_e;          // 前馈增益（可动态调整）
    float target_speed;  // 目标速度输入（来自小车）
    float error;
    float error_last;
    float intergral;
    float intergral_max;
    float intergral_min;
    float deadzone_min;
    float output;
    float output_min;
    float output_max;
} PID_Controller_t;

/* ----- 视觉通信相关 ----- */
// 浮点数字节共用体，用于串口接收解析
typedef union UnionFloat
{
    uint8_t Array[4];
    float FloatNum;
} UnionFloat_t;

// 视觉接收数据结构体
typedef struct
{
    float pos_x;
    float pos_y;
    float yaw;
    float laser_dis_x;
    float laser_dis_y;
    int16_t car_speed;
    uint8_t  car_mode;
} Receive_data;

// 通信帧常量定义
typedef struct {
    const uint8_t START[3];         // 启动帧: A5 FF 5A
    const uint8_t DATA_NUM;         // 应答帧: B6 01 6B
    const uint8_t CMD_HEAD;         // 帧头 A5
    const uint8_t CMD_TAIL;         // 帧尾 5A
    const uint8_t DATA1_HEAD;       // 第一组数据头 B6
    const uint8_t DATA1_TAIL;       // 第一组数据尾 6B
    const uint8_t CMD_OK;           // 完成指令 66
    const uint8_t CAR_DATA2_HEAD;   // 第二组数据头 C7
    const uint8_t CAR_DATA2_TAIL;   // 第二组数据尾 7C
    const uint8_t VF_TIMMEOUT;      // 接收超时时间，单位ms
    const uint8_t GIMBAL_RECV_HEAD; // D8 — 接收菜单数据帧头
    const uint8_t GIMBAL_RECV_TAIL; // 8D — 接收菜单数据帧尾
    const uint8_t GIMBAL_SEND_HEAD; // E9 — 发送数据到菜单帧头
    const uint8_t GIMBAL_SEND_TAIL; // 9E — 发送数据到菜单帧尾
    const uint8_t GIMBAL_TIMEOUT;   // 云台通讯超时时间，单位ms
} Vision_Frame_t;

/* ----- 云台通讯协议常量 ----- */
#define GIMBAL_FLOAT_COUNT  12  // 通讯数据: xPID(3) + yPID(3) + x前馈(6) = 12 个 float
#define GIMBAL_RECV_SIZE    50  // 接收帧: 1 + 12*4 + 1 = 50 字节
#define GIMBAL_SEND_SIZE    50  // 发送帧: 1 + 12*4 + 1 = 50 字节

/* 云台通讯数据索引 (与菜单 gimbalPid[] 一一对应)
 * [0]  pid_control_x.Kp
 * [1]  pid_control_x.Ki
 * [2]  pid_control_x.Kd
 * [3]  pid_control_y.Kp
 * [4]  pid_control_y.Ki
 * [5]  pid_control_y.Kd
 * [6]  kf_table[0] — mode 0 (默认)
 * [7]  kf_table[1] — mode 1 (圆弧)
 * [8]  kf_table[2] — mode 2 (第一个转弯)
 * [9]  kf_table[3] — mode 3 (直线)
 * [10] kf_table[4] — mode 4 (第二个转弯)
 * [11] kf_table[5] — mode 5
 */
enum {
    GIMBAL_IDX_X_KP = 0,
    GIMBAL_IDX_X_KI,
    GIMBAL_IDX_X_KD,
    GIMBAL_IDX_Y_KP,
    GIMBAL_IDX_Y_KI,
    GIMBAL_IDX_Y_KD,
    GIMBAL_IDX_KF_MODE0,
    GIMBAL_IDX_KF_MODE1,
    GIMBAL_IDX_KF_MODE2,
    GIMBAL_IDX_KF_MODE3,
    GIMBAL_IDX_KF_MODE4,
    GIMBAL_IDX_KF_MODE5,
};

// 视觉通信状态机
typedef enum
{
    VISION_IDLE,
    VISION_ACK,
    VISION_WAIT_CONDITION,
    VISION_SEND_OK,
    VISION_RECV_DATA2,
    VISION_FINISH
} Vision_State_t;

/* ----- 系统运行标志与配置 ----- */
typedef struct
{
    float initial_yaw;
    uint8_t first_frame_received;
    uint8_t data_valid;
    float flag_yaw;
    uint8_t stop_scan;
    uint8_t pid_change;
} Start_FlagStatus;

typedef struct
{
    uint8_t start_flag;
    uint8_t question_num;
    uint8_t scan;
    uint8_t scan_speed;
} scan_config;

/* ----- OLED显示配置 ----- */
typedef struct
{
    char *open_init_oled;
    uint8_t size_oled;
    uint8_t size_num;
    uint8_t size_float[2];

    char* start_flag;
    char* question_num;
    char* scan;

    char* pid_x_inr;
    char* pid_x_out;
    char* pid_y_out;

    char* yaw;
    char* init_yaw;

    uint8_t oled_transfer_flag;
} oled_config;

/* ==================================================
   4. 业务模块头文件
   ================================================== */
#include "OLED.h"
#include "QD4310.h"
#include "task_oled_and_init.h"
#include "task_key.h"
#include "task_receive_t_data.h"
#include "task_pid.h"
#include "task_scan.h"

/* ==================================================
   5. 全局变量统一extern声明
   所有跨文件访问的全局变量集中在此
   ================================================== */

/* ----- PID控制器实例 ----- */
extern PID_Controller_t pid_control_y;
extern PID_Controller_t pid_control_x;

/* ----- 视觉数据与通信 ----- */
extern Receive_data vision_data;
extern Vision_Frame_t Frame;
extern Vision_State_t vision_state;
extern UART_HandleTypeDef huart5;

/* ----- 云台通讯 (菜单 ↔ 云台 UART5) ----- */
extern float gimbal_comm_data[GIMBAL_FLOAT_COUNT];   // 与菜单交互的 12 个 float
extern volatile uint8_t gimbal_recv_flag;              // 接收完成标志
extern float kf_table[6];                              // X轴前馈系数表 (6 个 mode, 运行时可调)

/* ----- 系统状态与配置 ----- */
extern Start_FlagStatus flag;
extern scan_config scan_init;
extern float now_yaw;
extern uint8_t rx_yaw;
extern uint8_t car_rx_byte;

/* ----- 电机设备实例 ----- */
extern QD4310_t Motor_0;
extern QD4310_t Motor_1;

/* ----- OLED配置 ----- */
extern oled_config oled_init;

#ifdef __cplusplus
}
#endif

#endif /* __APP_COMMON_H */
