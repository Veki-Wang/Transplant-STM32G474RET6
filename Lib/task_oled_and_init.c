#include "app_common.h"

oled_config oled_init ={
    .open_init_oled = "wzl - hello",
    .size_oled = OLED_8X16,
    .question_num = "ques:",
    .start_flag = "star:",
    .scan = "scan:",
    .size_num = 1,
    .size_float[0] = 4,
    .size_float[1] = 2,
    .oled_transfer_flag = 0,
    .pid_x_inr = "XIN:",    
    .pid_x_out = "XOUT:",  
    .pid_y_out = "YOUT:",
    .yaw = "Yaw:",         
    .init_yaw = "Iyaw",
};



void oled_task_init(void) 
{
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, oled_init.open_init_oled, oled_init.size_oled);
    OLED_Update(); 
    HAL_Delay(500);
    OLED_Clear();

    OLED_Update(); 
}

void task_oled_and_init(void) 
{
    HAL_Delay(100);
    //OLED初始化参数
    oled_task_init();
    //使能UART3的接收中断,开启视觉的握手通信
    Vision_Task_Init();
    // HAL_UART_Receive_IT(&huart6, &rx_yaw, 1); 
    // HAL_UART_Receive_IT(&huart1, &car_rx_byte, 1);
}

void oled_task(void)
{
    // 检查是否接收到有效数据
    if((vision_data.pos_x != 0.0f || vision_data.pos_y != 0.0f) && scan_init.start_flag == 1) 
    {       
        // 有有效数据，显示PID信息
        OLED_Clear();
        //1:x_outer_out
        OLED_ShowString(0,  0,  oled_init.pid_x_out, oled_init.size_oled);
        OLED_ShowFloatNum(32, 0, pid_control_x.output, 3, 1, oled_init.size_oled);
        // OLED_ShowNum(96, 0, scan_init.start_flag, 1, oled_init.size_oled);  // 右侧显示整数1

        //2:y_outer_out
        OLED_ShowString(0,  16, oled_init.pid_y_out, oled_init.size_oled);
        OLED_ShowFloatNum(32, 16, pid_control_y.output, 3, 1, oled_init.size_oled);
        // OLED_ShowNum(96, 16, scan_init.scan, 1, oled_init.size_oled); 

        //3:vision_data.car_mode     vision_data.car_mode
        OLED_ShowSignedNum(0, 32, vision_data.car_mode, 3, oled_init.size_oled);
        OLED_ShowSignedNum(64, 32, vision_data.car_speed, 5, oled_init.size_oled);

        //4:视觉X,Y误差
        OLED_ShowFloatNum(0, 48, vision_data.pos_x, oled_init.size_float[0], oled_init.size_float[1], oled_init.size_oled);
        OLED_ShowFloatNum(64, 48, pid_control_x.Kf_e,oled_init.size_float[0], oled_init.size_float[1], oled_init.size_oled);
    
    } 
    else if((vision_data.pos_x == 0.0f && vision_data.pos_y == 0.0f) && scan_init.start_flag == 0 && flag.data_valid == 1)
    {
        // 显示无数据状态// 没有有效数据，显示状态信息
        OLED_Clear();
        OLED_ShowString(0, 16, oled_init.start_flag, oled_init.size_oled);
        OLED_ShowNum(48, 16, scan_init.start_flag, oled_init.size_num, oled_init.size_oled);
        
        OLED_ShowString(0, 0, oled_init.question_num, oled_init.size_oled);
        OLED_ShowNum(48, 0, scan_init.question_num, oled_init.size_num, oled_init.size_oled);
        
        OLED_ShowString(0, 32, oled_init.scan, oled_init.size_oled);
        OLED_ShowNum(48, 32, scan_init.scan, oled_init.size_num, oled_init.size_oled);

        OLED_ShowSignedNum(0, 48, vision_data.car_mode, 3, oled_init.size_oled);
        OLED_ShowSignedNum(64, 48, vision_data.car_speed, 5, oled_init.size_oled);

        OLED_ShowString(60, 0, "H_DATA", oled_init.size_oled);
    }
    else 
    {
        // 没有有效数据，显示状态信息
        OLED_Clear();
        OLED_ShowString(0, 16, oled_init.start_flag, oled_init.size_oled);
        OLED_ShowNum(48, 16, scan_init.start_flag, oled_init.size_num, oled_init.size_oled);
        
        OLED_ShowString(0, 0, oled_init.question_num, oled_init.size_oled);
        OLED_ShowNum(48, 0, scan_init.question_num, oled_init.size_num, oled_init.size_oled);
        
        OLED_ShowString(0, 32, oled_init.scan, oled_init.size_oled);
        OLED_ShowNum(48, 32, scan_init.scan, oled_init.size_num, oled_init.size_oled);

        OLED_ShowSignedNum(0, 48, vision_data.car_mode, 3, oled_init.size_oled);
        OLED_ShowSignedNum(64, 48, vision_data.car_speed, 5, oled_init.size_oled);

        OLED_ShowString(60, 0, "NO DATA", oled_init.size_oled);
    }
    
    OLED_Update();
}


