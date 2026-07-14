#ifndef __APP_COMMON_H
#define __APP_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

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

/* ===== Includes ===== */
#include "main.h"              
#include "huart3_base.h"       
#include "OLED.h"              
#include "QD4310.h"            

#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

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

#include "task_oled_and_init.h"
#include "task_key.h"
#include "task_receive_t_data.h"
#include "task_pid.h"
#include "task_scan.h"



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

/* ===== External Variables ===== */
extern Start_FlagStatus flag;
extern scan_config scan_init;
extern float now_yaw;
extern uint8_t rx_yaw;
extern uint8_t car_rx_byte;
extern QD4310_t Motor_0;
extern QD4310_t Motor_1;

typedef  struct
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

}oled_config;

extern oled_config oled_init;

extern PID_Controller_t pid_control_y;
extern PID_Controller_t pid_control_x;

#ifdef __cplusplus
}
#endif

#endif /* __APP_COMMON_H */
