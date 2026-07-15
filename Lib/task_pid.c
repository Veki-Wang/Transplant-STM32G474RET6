#include "app_common.h"


// Y轴单独的PID控制器实例
PID_Controller_t pid_control_y = {0};
PID_Controller_t pid_control_x = {0};

/* PID 默认参数 — 上电初始化 & 菜单下发时同步更新 */
PID_Params_t pid_params_y = { -0.8f, 0.0f, -0.01f };
PID_Params_t pid_params_x = {  0.36f, 0.0f,  0.0f };

/* ===================== X轴前馈系数表 (可通过菜单动态调参) ===================== */
float kf_table[6] = {
    0.0f,   // mode 0: 默认，关闭前馈
    0.6f,   // mode 1: 圆弧
    2.8f,   // mode 2: 第一个转弯
    0.45f,  // mode 3: 直线
    2.8f,   // mode 4: 第二个转弯
    3.0f    // mode 5
};

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
        const uint8_t table_size = sizeof(kf_table) / sizeof(kf_table[0]);
        uint8_t cur_mode = vision_data.car_mode;

        // 查表赋值，超出数组范围则使用默认值 0
        if(cur_mode < table_size)
        {
            pid_control_x.Kf_e = kf_table[cur_mode];
        }
        else
        {
            pid_control_x.Kf_e = 0.0f;
        }

        // 题目3 固定 PID 参数 (但允许菜单通过串口覆盖)
        // pid_control_x 的 Kp/Ki/Kd 由 gimbal_comm_data 通过 GimbalComm_ApplyRecvData 写入
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

        uint16_t motor_speed1 = (uint16_t)fabsf(pid_control_x.output);
        uint8_t  motor_dir1   = (pid_control_x.output >= 0.0f) ? 1 : 0;
        QD4310_SetSpeed(&Motor_0, (float)motor_speed1);

        uint16_t motor_speed2 = (uint16_t)fabsf(pid_control_y.output);
        uint8_t  motor_dir2   = (pid_control_y.output >= 0.0f) ? 1 : 0;
        QD4310_SetSpeed(&Motor_1, (float)motor_speed2);

        
        flag.data_valid = 0; // 外环计算完成，重置数据有效标志
    }
}

