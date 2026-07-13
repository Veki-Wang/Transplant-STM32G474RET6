#ifndef _TASK_PID_H_ 
#define _TASK_PID_H_

#include "main.h"
#include "task_receive_t_data.h"
#include <math.h>

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

extern PID_Controller_t pid_control_y;
extern PID_Controller_t pid_control_x;

extern float servo_pos;

void Control_PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float min_output, float max_output);
float PID_Compute(PID_Controller_t *pid,float y_error);
void PID_Control_Outer_Only(void);
void PID_Outer_Task(void);
void PID_Control_Outer_Only(void);
#endif // !_TASK_PID_H_