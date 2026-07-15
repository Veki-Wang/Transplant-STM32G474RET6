#include "app_common.h"       


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
    .GIMBAL_RECV_HEAD = 0xD8,  // 接收菜单帧头
    .GIMBAL_RECV_TAIL = 0x8D,  // 接收菜单帧尾
    .GIMBAL_SEND_HEAD = 0xE9,  // 发送到菜单帧头
    .GIMBAL_SEND_TAIL = 0x9E,  // 发送到菜单帧尾
    .GIMBAL_TIMEOUT   = 50,    // 云台通讯超时，单位ms
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

/* ===================== 云台通讯 (UART5 ↔ 菜单) ===================== */

/* 与菜单交互的 12 个 float: xPID(3) + yPID(3) + kf_table[0..5](6) */
float gimbal_comm_data[GIMBAL_FLOAT_COUNT] = {0.0f};

/* 接收完成标志, 主循环读取后清零 */
volatile uint8_t gimbal_recv_flag = 0;

/* 接收状态机 */
static uint8_t  gimbal_rx_buf[GIMBAL_RECV_SIZE];   /* 接收缓冲 50 字节 */
static uint8_t  gimbal_rx_idx = 0;                  /* 已填充字节数 */
static uint32_t gimbal_rx_last_tick = 0;            /* 最后收到字节的时间戳 */

/* 接收到的原始数据暂存 (12 个 float, 待 Apply 后正式写入 PID/kf_table) */
static float gimbal_rx_data[GIMBAL_FLOAT_COUNT] = {0.0f};

/* 发送缓冲: 50 字节 */
static uint8_t  gimbal_tx_buf[GIMBAL_SEND_SIZE];

/* 中断接收用的单字节缓冲 */
static uint8_t  gimbal_rx_byte;


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
    else if (huart == &huart5)
    {
        GimbalComm_FeedByte(gimbal_rx_byte);
        HAL_UART_Receive_IT(&huart5, &gimbal_rx_byte, 1);
    }
    // else if(huart == &huart1)
    // {
    //     car_data_feed_byte(car_rx_byte,&vision_data.car_speed, &vision_data.car_mode);
    //     HAL_UART_Receive_IT(&huart1, &car_rx_byte, 1);
    // }
}

/* ===================== 云台通讯 (UART5 ↔ 菜单) ===================== */

/**
 * @brief 初始化云台通讯: 启动 UART5 接收中断
 */
void GimbalComm_Init(void)
{
    gimbal_rx_idx  = 0;
    gimbal_rx_last_tick = 0;
    gimbal_recv_flag = 0;

    /* 启动 UART5 接收中断, 每次收 1 字节 */
    HAL_UART_Receive_IT(&huart5, &gimbal_rx_byte, 1);
}

/**
 * @brief 将当前 PID 和前馈系数打包成 E9 + 48 + 9E = 50 字节帧并发送到菜单
 *
 * 帧格式: E9 | float[12] 小端序 | 9E
 * 12 个 float: xPID(3) + yPID(3) + kf_table[0..5](6)
 */
void GimbalComm_Send(void)
{
    /* 先将当前值同步到 gimbal_comm_data */
    gimbal_comm_data[GIMBAL_IDX_X_KP] = pid_control_x.Kp;
    gimbal_comm_data[GIMBAL_IDX_X_KI] = pid_control_x.Ki;
    gimbal_comm_data[GIMBAL_IDX_X_KD] = pid_control_x.Kd;
    gimbal_comm_data[GIMBAL_IDX_Y_KP] = pid_control_y.Kp;
    gimbal_comm_data[GIMBAL_IDX_Y_KI] = pid_control_y.Ki;
    gimbal_comm_data[GIMBAL_IDX_Y_KD] = pid_control_y.Kd;
    for (uint8_t i = 0; i < 6; i++)
    {
        gimbal_comm_data[GIMBAL_IDX_KF_MODE0 + i] = kf_table[i];
    }

    gimbal_tx_buf[0] = Frame.GIMBAL_SEND_HEAD;              /* 帧头 E9 */

    /* 打包 12 个 float, 小端序 */
    for (uint8_t i = 0; i < GIMBAL_FLOAT_COUNT; i++)
    {
        UnionFloat_t f;
        f.FloatNum = gimbal_comm_data[i];
        gimbal_tx_buf[1 + i * 4 + 0] = f.Array[0];
        gimbal_tx_buf[1 + i * 4 + 1] = f.Array[1];
        gimbal_tx_buf[1 + i * 4 + 2] = f.Array[2];
        gimbal_tx_buf[1 + i * 4 + 3] = f.Array[3];
    }

    gimbal_tx_buf[GIMBAL_SEND_SIZE - 1] = Frame.GIMBAL_SEND_TAIL; /* 帧尾 9E */

    /* 阻塞发送 */
    HAL_UART_Transmit(&huart5, gimbal_tx_buf, GIMBAL_SEND_SIZE, 100);
}

/**
 * @brief 从 UART 中断回调传入每一个收到的字节 (接收状态机)
 *
 * 状态机流程:
 *   1. 超时保护: 上一字节距今超过 GIMBAL_TIMEOUT → 丢弃半截包
 *   2. State 0 (未找到帧头): 等待 0xD8
 *   3. State 1 (正在填充): 逐字节存入 gimbal_rx_buf
 *   4. State 2 (收满 50 字节): 校验帧尾 0x8D
 *      - 成功 → 解析 12 个 float 到 gimbal_rx_data, 置 gimbal_recv_flag
 *      - 失败 → 在已收数据中搜索下一个帧头, 移位或复位
 */
void GimbalComm_FeedByte(uint8_t ch)
{
    uint32_t now = HAL_GetTick();

    /* --- 超时保护 --- */
    if (gimbal_rx_idx > 0 && (now - gimbal_rx_last_tick) > Frame.GIMBAL_TIMEOUT)
    {
        gimbal_rx_idx = 0;
    }
    gimbal_rx_last_tick = now;

    /* --- State 0: 寻找帧头 D8 --- */
    if (gimbal_rx_idx == 0)
    {
        if (ch == Frame.GIMBAL_RECV_HEAD)  /* 0xD8 */
        {
            gimbal_rx_buf[0] = ch;
            gimbal_rx_idx = 1;
        }
        return;
    }

    /* --- State 1: 填充包体 --- */
    gimbal_rx_buf[gimbal_rx_idx++] = ch;

    if (gimbal_rx_idx < GIMBAL_RECV_SIZE)  /* 50 */
    {
        return;   /* 还没收满 */
    }

    /* --- State 2: 收满 50 字节, 校验帧尾 --- */
    if (gimbal_rx_buf[GIMBAL_RECV_SIZE - 1] == Frame.GIMBAL_RECV_TAIL)  /* 0x8D */
    {
        /* 解析 12 个 float */
        for (uint8_t i = 0; i < GIMBAL_FLOAT_COUNT; i++)
        {
            UnionFloat_t f;
            f.Array[0] = gimbal_rx_buf[1 + i * 4 + 0];
            f.Array[1] = gimbal_rx_buf[1 + i * 4 + 1];
            f.Array[2] = gimbal_rx_buf[1 + i * 4 + 2];
            f.Array[3] = gimbal_rx_buf[1 + i * 4 + 3];
            gimbal_rx_data[i] = f.FloatNum;
        }

        gimbal_recv_flag = 1;   /* 通知主循环: 有新数据 */
        gimbal_rx_idx = 0;
        return;
    }

    /* --- 帧尾错误: 在已收缓冲区里搜索下一个 D8 --- */
    uint8_t k;
    for (k = 1; k < GIMBAL_RECV_SIZE; k++)
    {
        if (gimbal_rx_buf[k] == Frame.GIMBAL_RECV_HEAD)
            break;
    }

    if (k < GIMBAL_RECV_SIZE)
    {
        /* 找到了, 把从 k 开始的数据搬到缓冲开头 */
        uint8_t n = GIMBAL_RECV_SIZE - k;
        for (uint8_t i = 0; i < n; i++)
        {
            gimbal_rx_buf[i] = gimbal_rx_buf[k + i];
        }
        gimbal_rx_idx = n;
    }
    else
    {
        /* 整个缓冲都没有 D8, 彻底复位 */
        gimbal_rx_idx = 0;
    }
}

/**
 * @brief 在主循环中检测到 gimbal_recv_flag != 0 后调用,
 *        将收到的数据写入 pid_control_x, pid_control_y, kf_table[]
 *
 * 12 个 float 的映射:
 *   [0..2]  → pid_control_x.Kp, Ki, Kd
 *   [3..5]  → pid_control_y.Kp, Ki, Kd
 *   [6..11] → kf_table[0..5]
 */
void GimbalComm_ApplyRecvData(void)
{
    /* 写入运行时 PID 结构体 */
    pid_control_x.Kp = gimbal_rx_data[GIMBAL_IDX_X_KP];
    pid_control_x.Ki = gimbal_rx_data[GIMBAL_IDX_X_KI];
    pid_control_x.Kd = gimbal_rx_data[GIMBAL_IDX_X_KD];

    pid_control_y.Kp = gimbal_rx_data[GIMBAL_IDX_Y_KP];
    pid_control_y.Ki = gimbal_rx_data[GIMBAL_IDX_Y_KI];
    pid_control_y.Kd = gimbal_rx_data[GIMBAL_IDX_Y_KD];

    /* 同步到参数变量，保证 pid_params 始终是最新的菜单值 */
    pid_params_x.kp = gimbal_rx_data[GIMBAL_IDX_X_KP];
    pid_params_x.ki = gimbal_rx_data[GIMBAL_IDX_X_KI];
    pid_params_x.kd = gimbal_rx_data[GIMBAL_IDX_X_KD];

    pid_params_y.kp = gimbal_rx_data[GIMBAL_IDX_Y_KP];
    pid_params_y.ki = gimbal_rx_data[GIMBAL_IDX_Y_KI];
    pid_params_y.kd = gimbal_rx_data[GIMBAL_IDX_Y_KD];

    for (uint8_t i = 0; i < 6; i++)
    {
        kf_table[i] = gimbal_rx_data[GIMBAL_IDX_KF_MODE0 + i];
    }

    /* 同步到 gimbal_comm_data, 供下次发送使用 */
    for (uint8_t i = 0; i < GIMBAL_FLOAT_COUNT; i++)
    {
        gimbal_comm_data[i] = gimbal_rx_data[i];
    }

    gimbal_recv_flag = 0;
}

