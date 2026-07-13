#include "task_receive_t_data.h"


/* ===================== 帧头帧尾的所有的变量 =====================*/
Vision_Frame_t Frame = {
    .START    = {0xA5, 0xFF, 0x5A},
    .DATA_NUM = 10,
    .CMD_HEAD = 0xA5,
    .CMD_TAIL = 0x5A,
    .DATA1_HEAD = 0xB6,
    .DATA1_TAIL = 0x6B,
    .CMD_OK     = 0x66,
    .CAR_DATA2_HEAD = 0xC7,
    .CAR_DATA2_TAIL = 0x7C,
    .VF_TIMMEOUT = 50, // 接收超时时间，单位ms
};

/*===================== 全局初始化参数 =====================*/
Receive_data vision_data = {0};
Vision_State_t vision_state = VISION_IDLE;// 通信状态机初始状态,就是空闲状态
static uint8_t rx_byte = 0;
static uint8_t  rx_buf[10];  //串口中断的缓冲帧
static uint8_t  vf_idx = 0;     // 当前已填充到 vf_buf 的字节数

static uint8_t car_buf[5];       // 串口接收到的小车的速度数据
static uint8_t  car_idx = 0;     // 当前已填充到 car_buf 的字节数

static uint32_t vf_last_tick = 0;   // 最后一次收到字节的时间


/* 初始化发送FF进行启动*/
void Vision_Task_Init(void)
{
    
    Vision_Send_Start();
    // 开启串口接收中断
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    // 启动通信
}
// ===================== 主函数调用的状态机切换逻辑 =====================
void Vision_Task(void)
{
    // 等待条件满足 → 发送完成指令
    if(scan_init.start_flag == 1 && flag.stop_scan == 1)
    {

        static uint8_t laser_locked = 0; // 激光锁定标志，初始为未锁定
        if (!laser_locked)
        {
            static uint8_t stable_cnt = 0;
            // 连续10帧稳定(≈150ms) → 开光锁死
            if ((vision_data.pos_x >= -9.0f && vision_data.pos_x <= 9.0f)  && (vision_data.pos_y >= -4.0f && vision_data.pos_y <= 4.0f))
            {
                stable_cnt++;
                if (stable_cnt >= 5)
                {
                    laser_locked = 1;
                    LASER_ON(); // 激光打开，永不关闭 (通过硬件复位清零)
                    vision_state = VISION_WAIT_CONDITION;
                }
            }
            else
            {
                stable_cnt = 0;
            }
        }
    }
    if(vision_state == VISION_WAIT_CONDITION)
    {
        Vision_Send_OK();//发送A5 66 5A
        vision_state = VISION_RECV_DATA2;
        if(scan_init.question_num == 3)
        {
            flag.pid_change = 1;
        }
    }
}

static inline void car_data_feed_byte(uint8_t ch,int16_t *car_speed, uint8_t *car_mode)
{
    uint32_t now = HAL_GetTick();
    if (car_idx > 0 && (now - vf_last_tick) > Frame.VF_TIMMEOUT) {
        car_idx = 0;
    }
    vf_last_tick = now;

   if (car_idx == 0) {
        if (ch == Frame.CAR_DATA2_HEAD) {
            car_buf[0] = ch;
            car_idx = 1;
        }
        // 非 C7 一律丢弃, 继续找
        return;
    }
    car_buf[car_idx++] = ch;

    if (car_idx < 5) {
        return;    // 还没收满, 继续等下一字节
    }

    // ========== 状态 3: 收满 5 字节, 校验帧尾 ==========
    if (car_buf[4] == Frame.CAR_DATA2_TAIL) {
        vision_data.car_speed = (car_buf[2] << 8) | car_buf[1]; // 低字节在前
        vision_data.car_mode = car_buf[3]; // 高字节在后
        car_idx = 0;
        return;
    }

    // ★ 帧尾错误 = 同步丢失, 必须重找帧头 ★
    // 关键: 不能简单清零, 要在已收 10 字节的 [1..9] 里找 B6
    // 否则可能错过下一个真正的帧头
    uint8_t k;
    for (k = 1; k < 5; k++) {
        if (car_buf[k] == Frame.CAR_DATA2_HEAD) break;
    }

    if (k < 5) {
        // 在 buffer 中间找到了新 C7, 把从它开始的字节搬到 buffer 开头
        uint8_t n = 5 - k;
        for (uint8_t i = 0; i < n; i++) {
            car_buf[i] = car_buf[k + i];
        }
        car_idx = n;     // 保留这些字节作为新帧开头
    } else {
        // 整个 buffer 里一个 7C 都没有, 彻底重来
        car_idx = 0;
    }
}


static inline void vision_feed_byte(uint8_t ch,float *dx, float *dy)
{
    // --- 超时保护: 上一字节距离现在太久, 说明之前的半截包作废 ---
    uint32_t now = HAL_GetTick();
    if (vf_idx > 0 && (now - vf_last_tick) > Frame.VF_TIMMEOUT) {
        vf_idx = 0;
    }
    vf_last_tick = now;

    // ========== 状态 1: 还没找到帧头 ==========
    if (vf_idx == 0) {
        if (ch == Frame.DATA1_HEAD) {
            rx_buf[0] = ch;
            vf_idx = 1;
        }
        // 非 B6 一律丢弃, 继续找
        return;
    }

    // ========== 状态 2: 正在填充包体 ==========
    rx_buf[vf_idx++] = ch;

    if (vf_idx < Frame.DATA_NUM) {
        return;    // 还没收满, 继续等下一字节
    }

    // ========== 状态 3: 收满 10 字节, 校验帧尾 ==========
    if (rx_buf[Frame.DATA_NUM - 1] == Frame.DATA1_TAIL) {
        UnionFloat_t fx, fy;
        for (uint8_t i = 0; i < 4; i++) fx.Array[i] = rx_buf[1 + i];
        for (uint8_t i = 0; i < 4; i++) fy.Array[i] = rx_buf[5 + i];

        *dx = fx.FloatNum;
        *dy = fy.FloatNum;

        if (vision_data.pos_x != 0.0f || vision_data.pos_y != 0.0f) {
            flag.data_valid = 1;
            flag.stop_scan = 1;
            // scan_init.scan   = 0;
            if (!flag.first_frame_received) {
                flag.initial_yaw = now_yaw;
                flag.first_frame_received = 1;
            }
        }
        flag.data_valid = 1;
        vf_idx = 0;    // 准备接收下一帧
        return;
    }

    // ★ 帧尾错误 = 同步丢失, 必须重找帧头 ★
    // 关键: 不能简单清零, 要在已收 10 字节的 [1..9] 里找 B6
    // 否则可能错过下一个真正的帧头
    uint8_t k;
    for (k = 1; k < Frame.DATA_NUM; k++) {
        if (rx_buf[k] == Frame.DATA1_HEAD) break;
    }

    if (k < Frame.DATA_NUM) {
        // 在 buffer 中间找到了新 B6, 把从它开始的字节搬到 buffer 开头
        uint8_t n = Frame.DATA_NUM - k;
        for (uint8_t i = 0; i < n; i++) {
            rx_buf[i] = rx_buf[k + i];
        }
        vf_idx = n;     // 保留这些字节作为新帧开头
    } else {
        // 整个 buffer 里一个 B6 都没有, 彻底重来
        vf_idx = 0;
    }
}

// ====================== HAL 回调 ======================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        // 只在 VISION_ACK 状态下解析, 其它状态丢弃
        if (vision_state == VISION_ACK || vision_state == VISION_RECV_DATA2) {
            vision_feed_byte(rx_byte,&vision_data.pos_x,&vision_data.pos_y);
        } else {
            vf_idx = 0;   // 非接收状态, 复位状态机
        }
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
    // else if(huart == &huart1)
    // {   
    //     car_data_feed_byte(car_rx_byte,&vision_data.car_speed, &vision_data.car_mode);
    //     HAL_UART_Receive_IT(&huart1, &car_rx_byte, 1);
    // }
}

