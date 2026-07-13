#include "task_pid.h"
#include <math.h>

// Y轴单独的PID控制器实例
PID_Controller_t pid_control_y = {0};
PID_Controller_t pid_control_x = {0};

/*PID的算法实现*/
/******************************************************************************************/
void Control_PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float min_output, float max_output)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->Kf_e = 0.0f;        // 默认关闭前馈
    pid->target_speed = 0.0f;
    
    pid->error = 0.0f;
    pid->error_last = 0.0f;
    pid->intergral = 0.0f;

    pid->output = 0.0f;
    pid->intergral_max = 130;//max_output * 0.8;
    pid->intergral_min = -130;//min_output * 0.8;
    pid->output_min = min_output;
    pid->output_max = max_output;
    pid->deadzone_min = 5.0f;
}

/*只使用了微分先行的PID，*/
float PID_Compute(PID_Controller_t *pid,float y_error)
{
    pid->error = y_error;
    pid->target_speed = vision_data.car_speed;
    /*只有当像素大过5像素的时候才允许积分*/
    if(fabs(pid->error) > pid->deadzone_min)
    {
    pid->intergral += pid->error;//没用的到积分，使用看一下是否能消掉稳态误差
    if(pid->intergral > pid->intergral_max){pid->intergral = pid->intergral_max;}
    else if(pid->intergral < pid->intergral_min){pid->intergral = pid->intergral_min;}
    }
    else {
        pid->intergral *= 0.95f;
    }

    float differential = pid->error - pid->error_last;

    // 前馈项：增益 × 目标速度
    float feedforward = pid->Kf_e * pid->target_speed;

    pid->output = pid->Kp * pid->error + 
                  pid->Ki * pid->intergral +
                  pid->Kd * differential + 
                  feedforward;

    //做了一个输出的限幅和一个积分限幅，积分能输出的力只能贡献输出的百分之20%，先试一下
    if (pid->output > pid->output_max) 
        pid->output = pid->output_max; 
    else if (pid->output < pid->output_min) 
        pid->output = pid->output_min;
    
    pid->error_last = pid->error;
    return pid->output;
}

/******************************************************************************************/

void PID_Outer_Task(void)
{   
    /*前两题是只靠外环来解决*/
    PID_Compute(&pid_control_y, vision_data.pos_y);
    if(scan_init.question_num == 1 || scan_init.question_num == 2)
    {
        pid_control_x.Kf_e = 0.0f;
        PID_Compute(&pid_control_x, vision_data.pos_x);

    }
    else if(scan_init.question_num == 3 && flag.pid_change == 1)
    {
        if(vision_data.car_mode == 1)//圆弧，车速较慢，前馈增益可以适当大一些，提升响应速度
        {
            pid_control_x.Kf_e = 0.6f;//0.8f; // 根据经验调整前馈增益，过大可能引起震荡，过小可能响应不足
        }
        else if(vision_data.car_mode == 2)//第一个转弯
        {
            pid_control_x.Kf_e = 2.8f;//0.4f;
        }
        else if(vision_data.car_mode == 3)//直线，车速较快，前馈增益可以适当小一些，避免过冲
        {
            pid_control_x.Kf_e = 0.45f;
        }
        else if(vision_data.car_mode == 4)//第二个转弯，车速较慢，前馈增益可以适当大一些，提升响应速度
        {
            pid_control_x.Kf_e = 2.8f;//0.8f;
        }
        else if(vision_data.car_mode == 5)//第二个转弯，车速较慢，前馈增益可以适当大一些，提升响应速度
        {
            pid_control_x.Kf_e = 3.0f;//0.8f;
        }
        else//其他情况（如停车等待等），关闭前馈，避免不必要的输出
        {
            pid_control_x.Kf_e = 0.0f;
        }
            pid_control_x.Kp = 0.33f;//0.98f; 
            pid_control_x.Ki = 0.0f;
            pid_control_x.Kd = 0.3f;
    }
    PID_Compute(&pid_control_x, vision_data.pos_x);
}

void Set_Servo_Y_Pos(float pos, float pos_delta)
{
    if (pos > 1000.0f) pos = 1000.0f;
    if (pos < 400.0f) pos = 400.0f;
    //WritePosEx(1, (int16_t)pos, 90, 0);  
}

float servo_pos = 0.0f;
void PID_Control_Outer_Only(void)// 这个函数只计算外环PID，并直接控制电机速度，适合初期调试外环参数
{
    if (!flag.first_frame_received)return;
    if(flag.data_valid == 1)
    {
        PID_Outer_Task();

        uint16_t motor_speed = (uint16_t)fabsf(pid_control_x.output);
        uint8_t  motor_dir   = (pid_control_x.output >= 0.0f) ? 1 : 0;
        //Emm_V5_Vel_Control(1, motor_dir, motor_speed, 0, 0);

        static float servo_pos_y = 500.0f;
        servo_pos_y += pid_control_y.output;
        servo_pos = servo_pos_y;
        //Set_Servo_Y_Pos(servo_pos_y, pid_control_y.output);
        flag.data_valid = 0; // 外环计算完成，重置数据有效标志
    }
}

