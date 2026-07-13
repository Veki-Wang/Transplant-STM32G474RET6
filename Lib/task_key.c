#include "task_key.h"

static volatile unsigned char Key_Code = 0;
/**
 * @brief 外部调用函数：获取非连续键值（松手后返回一次）
 * @return unsigned char 按键键值（0=无按键，1=KEY1，2=KEY2）
 */
unsigned char Key_GetCode(void)
{
    volatile unsigned char TempCode = Key_Code;
    Key_Code = 0;  // 读取后清零，确保只返回一次
    return TempCode;
}

/**
  * 函数名：Key_Get
  * 功能：读取当前按键的电平状态，返回实时键码
  * 参数：无
  * 返回值：unsigned char - 实时键码（0=无按键，1=KEY1，2=KEY2）
  * 说明：适配F4 HAL库电平读取API
  */
unsigned char Key_Get(void)
{
    volatile unsigned char CurrentKey = 0; // 默认无按键

    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
        CurrentKey = 1;  // KEY1按下
    }
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
        CurrentKey = 2;  // KEY2按下
    }
    if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET)
    {
        CurrentKey = 3;  // KEY3按下
    }
    return CurrentKey;
}

/**
  * 函数名：Key_LoopDetect
  * 功能：循环检测按键状态，实现消抖与松手检测，更新全局键码
  * 参数：无
  * 返回值：无
  * 说明：需在定时器中断中调用（推荐10ms调用一次），保留原逻辑无修改
  */
void Key_LoopDetect(void)
{
    static unsigned char last = 0;
    unsigned char now = Key_Get();
    // 松手才触发，完美消抖
    if (last != 0 && now == 0) {
        Key_Code = last;
    }

    last = now;
}


// void key_task(void)
// {
//     Key_LoopDetect();
//     uint8_t keyValue = Key_GetCode();  

//     if (keyValue == 1)
//     {
//     scan_init.question_num++;
//     if (scan_init.question_num > 3)
//     {
//         scan_init.question_num = 1;
        
//     }
//     }
//     else if (keyValue == 2)
//     {
//         scan_init.start_flag = !scan_init.start_flag;
//     }
//     else if (keyValue == 3)
//     {
//         scan_init.scan++;
//         if (scan_init.scan > 2)
//         {
//             scan_init.scan = 0;
//         }
//     }
// }
