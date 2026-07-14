#include "app_common.h"
// ===================== 内部工具函数：串口发送单字节 =====================
void USART3_Send_Byte(uint8_t byte)
{
    HAL_UART_Transmit(&huart3, &byte, 1, 10);
}

// ===================== 内部：发送启动帧 A5 FF 5A=====================
void Vision_Send_Start(void)
{
    for(uint8_t i=0; i<3; i++)
        USART3_Send_Byte(Frame.START[i]);
    vision_state = VISION_ACK;
}

// ===================== 内部：发送完成指令 A5 66 5A=====================
void Vision_Send_OK(void)
{
    USART3_Send_Byte(Frame.CMD_HEAD);
    USART3_Send_Byte(Frame.CMD_OK);
    USART3_Send_Byte(Frame.CMD_TAIL);
    vision_state = VISION_RECV_DATA2;
}
