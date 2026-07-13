#include "task_scan.h"

// - 如果 SCAN==1 或 SCAN==2 且误差为 0 则进入扫描模式。
// - 如果 SCAN==1/2 且误差不为 0，则停止扫描（设置 SCAN=0），进入 PID 跟踪。
// - 如果 SCAN==0，则无论误差是否为0，都不进入扫描（误差为0代表已对准），进入 PID 跟踪后面不会在进入扫描模式。

void task_scan(void)
{ 
    // if(flag.stop_scan == 0)
    // {
    //     if (scan_init.start_flag == 0)
    //     {
    //         Emm_V5_Stop_Now(1, 0);
    //         return;
    //     }
    //     else if ((scan_init.scan == 1 || scan_init.scan == 2) && scan_init.start_flag == 1)
    //     {
    //         // 要求扫描且视觉未识别到目标 -> 执行扫描
    //         Serial_Scan_Mode();
    //     }
    // }
}
void Serial_Scan_Mode(void)
{
    // if (scan_init.scan == 1)
    // {
    //     // X轴顺时针旋转
    //     Emm_V5_Vel_Control(1, 1, scan_init.scan_speed, 0, 0); 
    // }
    // else if (scan_init.scan == 2)
    // {
    //     // X轴逆时针旋转
    //     Emm_V5_Vel_Control(1, 0, scan_init.scan_speed, 0, 0); 
    // }
}
