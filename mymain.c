#include "mymain.h"
#include "coop.h" // 【必须引入】，否则隐式声明报错
#include <stdlib.h>
#include <math.h>

// ====================================================================
// 全局变量本体定义 (分配内存空间)
// ====================================================================
float pitch, roll, yaw;
float g_Yaw_Offset = 0.0f; // 航向角软件偏置量 (本体)
float g_Raw_Yaw = 0.0f;    /* [2026-06-10 诊断] DMP原始yaw(未解卷绕), 供遥测对比 */
uint32_t g_DMP_Elapsed_Ticks = 1; /* [2026-06-12 诊断] 两次成功DMP读取之间跨过的10ms拍数 */
uint32_t g_DMP_Drain_Count = 1;   /* [2026-06-12 诊断] 本次控制周期实际读出的DMP FIFO包数 */

// yaw 解卷绕状态机
static float s_prev_raw_yaw = 0.0f;
static bool  s_yaw_first = true;

void Yaw_Integrator_Reset(void) {
    s_yaw_first = true;
    s_prev_raw_yaw = 0.0f;
}

// Normalize_Angle 定义已移至 pid.c (公共模块，debug/main 构建共用)
// ====================================================================
// 原 empty.c 中的主逻辑，现封装为 mymain
// ====================================================================
void mymain(void)
{
    /* ====================================================================
     * 内核心跳强启矩阵 (解决 g_Flag_10ms 恒 0 死锁)
     * ==================================================================== */
    DL_SYSTICK_enableInterrupt(); // 使能滴答定时器产生异常中断
    DL_SYSTICK_enable();          // 启动滴答定时器核心计数器

    // 1. 初始化 SysConfig 统筹的所有时钟与外设底座
    SYSCFG_DL_init();

    // --- 恢复 2 车通信接收逻辑 ---
    // 清除可能积压的中断标志位，随后在 NVIC 矩阵中使能 UART_COOP。
    DL_UART_Main_clearInterruptStatus(UART_COOP_INST, DL_UART_MAIN_INTERRUPT_RX);
    
    // 1. 配置右轮编码器组 (GPIOB_SENSOR) 的中断优先级为 0 (最高)
    NVIC_SetPriority(GPIO_SENSOR_INT_IRQN, 1);

    // 2. 配置左轮编码器组 (GPIOA_LEFT) 的中断优先级为 0 (最高)
    NVIC_SetPriority(GPIO_LEFT_INT_IRQN, 1);

    // 3. 配置树莓派视觉串口 (UART_PI) 的中断优先级为 2
    NVIC_SetPriority(UART_PI_INST_INT_IRQN, 2);    
    // 1优先级：系统滴答定时器，维持 10ms 绝对时基
    NVIC_SetPriority(SysTick_IRQn, 0); // 视底层库配置方式而定
    
    // 2优先级 (最低)：UART 串口收发 (视觉、VOFA、HMI)
    NVIC_SetPriority(UART_BLE_INST_INT_IRQN, 2);
    NVIC_SetPriority(UART_HMI_INST_INT_IRQN, 2);

    // 🔻 [Init 1]: 硬件验证与唤醒
    while(MPU6050_Init() != 0) {
        printf("[ERROR] MPU6050 I2C Offline. Check PA0(SDA)/PA1(SCL) wiring.\r\n");
        delay_cycles(10000); 
    }
    printf("[OK] MPU6050 Physical Link Established.\r\n");
    
    // 🔻 [Init 2]: DMP固件强启 (二车通信及运动控制的核心)
    while(mpu_dmp_init() != 0) {
        printf("DMP Library Loading Failed! Check SCL/SDA Pull-up.\r\n");
        delay_cycles(10000); 
    }
    
    printf("MPU6050 DMP System Online!\r\n");
    // --- 现在可以开始 PC 输出了 ---
    printf("LITO System Online. Redirection to PC Successful!\r\n");

    // 初始姿态快照 (重试至 DMP FIFO 就绪)
    {
        float raw_yaw_init;
        while (mpu_dmp_get_data(&pitch, &roll, &raw_yaw_init) != 0) {
            delay_ms(10);
        }
        yaw = raw_yaw_init;
        s_prev_raw_yaw = raw_yaw_init;
        s_yaw_first = false;
    }
    
    /* ====================================================================
     * 2. 硬件引擎防呆启动算子 (Fool-proof Engine Start)
     * 物理映射：强制向 Timer 控制寄存器写入使能位，确保 PWM 与 QEI 绝对起振
     * ==================================================================== */
    DL_TimerA_startCounter(PWM_LEFT_INST);   // 强制启动左轮 PWM (TIMA0)
    DL_TimerA_startCounter(PWM_RIGHT_INST);  // 强制启动右轮 PWM (TIMA1)
    
    /* ====================================================================
     * 3. 核心中断总控分闸矩阵 (NVIC Level)
     * ==================================================================== */
    NVIC_EnableIRQ(UART_HMI_INST_INT_IRQN);   
    NVIC_EnableIRQ(UART_PI_INST_INT_IRQN);    
    NVIC_EnableIRQ(UART_BLE_INST_INT_IRQN);   
    NVIC_EnableIRQ(UART_COOP_INST_INT_IRQN); 
     
    /* ====================================================================
     * GPIO 中断打通：严格调用 SysConfig 生成的宏
     * ==================================================================== */
    NVIC_EnableIRQ(GPIO_LEFT_INT_IRQN);  // 使能 PA29 (左轮) 中断
    NVIC_EnableIRQ(GPIO_SENSOR_INT_IRQN);  // 使能 PB12 (右轮) 和 PB19 (载药) 中断
    
    /* ====================================================================
     * 4. 致命修复：Cortex-M0+ 全局中断总闸 (Core Level)
     * 物理映射：清除 PRIMASK 寄存器，允许 CPU 响应所有外设中断
     * ==================================================================== */
    __enable_irq();  // 【必须加此句，否则系统等于脑死亡】

    /* ====================================================================
     * 5. 硬件生命体征初检与待机状态注入
     * ==================================================================== */
    LED_ALL_OFF();
    LED_GREEN_ON();
    
    // 初始化确保双轮处于强刹车锁死状态
    Motor_SetPWM(0, 0);
    Motor_SetPWM(1, 0);

    // ====================================================================
    // 1. 内环速度 PI 环 (左右轮独立)
    // ====================================================================
   // PID_Init(&pid_speed_L, 7.5f, 1.5f, 0.0f, 900.0f, 800.0f, 0.2f, 150.0f);
    //PID_Init(&pid_speed_R, 7.5f, 1.5f, 0.0f, 900.0f, 800.0f, 0.2f, 150.0f);
    //针对0.12mps
    //0.08mps更新（硬件老化）制动不一致。
    PID_Init(&pid_speed_L, 40.0f, 1.0f, 0.0f, 900.0f, 800.0f, 0.2f, 150.0f);
    PID_Init(&pid_speed_R, 40.0f, 1.0f, 0.0f, 900.0f, 800.0f, 0.2f, 150.0f);

    // ====================================================================
    // 2. 外环 视觉位置 PD 环 (树莓派巡红线)0.0028
    // ====================================================================
    PID_Init(&pid_vision, 0.05f, 0.0f, 0.054f, 30.0f, 0.0f, 0.5f, 0.0f);//适用0.08
    //PID_Init(&pid_vision, 0.0022f, 0.0f, 0.012f, 15.0f, 0.0f, 0.5f, 0.0f);//适用0.12


    // ====================================================================
    // 3. 外环 陀螺仪角度 PD 环 (MPU6500)
    // ====================================================================
    //PID_Init(&pid_angle_Z, 0.26f, 0.0f, 0.04f, 30.0f, 0.0f, 2.0f, 0.0f);//适用0.12
    /* [2026-06-12 A/D过冲修复]
     * 调试层 mymain4.c 在 PG=0.25/DG=0.03 下使用 out_max=30，实测无超调。
     * 主层原 out_max=60 会在同一 P/D 下给出两倍差速力矩，放大落地摩擦和刹车差异，
     * 导致 90 度转弯出现概率性超调/回摆。此处向调试层收敛，先保证同参数同限幅。
     */
    PID_Init(&pid_angle_Z, 0.5f, 0.0f, 0.035f, 30.0f, 0.0f, 2.0f, 0.0f);

    /* ==================================================================== */
    /* 3b. 反向循迹独立陀螺仪 PD 环 (参数在 FSM_Trigger_Reverse_Start 装载) */
    /* ==================================================================== */
    PID_Init(&pid_angle_Z_rev, g_Rev_Gyro_Kp, 0.0f, g_Rev_Gyro_Kd,
             15.0f, 0.0f, 0.2f, 0.0f);
// M/T法10ms的量化抖动大概是1~3个脉冲，这里阈值设为 2.0f 是绝佳起点
//AKF_Init(&akf_speed_L, 2.0f); 
//AKF_Init(&akf_speed_R, 2.0f);

    FSM_Init_first();

    /* ====================================================================
     * 主循环时间片调度矩阵 (Task Scheduler)
     * ==================================================================== */
    // 分频计数器
    static uint8_t tick_50ms  = 0;
    static uint8_t tick_300ms = 0;
    
    // 状态缓存器 (用于优化屏幕刷新带宽，仅改变时发送)
    static SystemState_t last_hmi_state = STATE_IDLE; 
    static uint8_t       last_hmi_room  = 0xFF; // 赋一个非法初值，保证开机必刷一次
// 【新增】：DMP 读取通知旗语
volatile bool g_Flag_Read_DMP = false;
    // 在单片机初始化后调用
    HMI_Send_Raw_String("t_stat.txt=\"ON_THEBOARD\"");

    while (1)
    {
        // -----------------------------------------------------------
        // [Level 0]: 全速轮询层 (无阻塞，极速吃掉外设指令)
        // -----------------------------------------------------------
        /* [2026-06-01] UART_BLE 收数轮询 — ISR 暂不触发, 直接读 RIS 替代
         * 必须在 VOFA_Parse_Command 之前, 确保 rx_frame_ready 被置位
         * [2026-06-09] 移除LED诊断翻转: 红灯闪灭干扰用户对状态机LED的判断 */
        {
            uint32_t ris = DL_UART_Main_getEnabledInterruptStatus(
                               UART_BLE_INST, DL_UART_INTERRUPT_RX);
            if (ris & DL_UART_INTERRUPT_RX) {
                DL_UART_Main_clearInterruptStatus(UART_BLE_INST,
                                                  DL_UART_INTERRUPT_RX);
                while (!DL_UART_Main_isRXFIFOEmpty(UART_BLE_INST)) {
                    uint8_t c = DL_UART_Main_receiveData(UART_BLE_INST);
                    VOFA_Receive_Byte_ISR(c);
                }
            }
        }

        VOFA_Parse_Command();
        VOFA_Tx_Pump();
        HMI_Parse_Command();  
if (g_Flag_Read_DMP == 1 && !FSM_Is_Turn_Critical_Window()){
g_Flag_Read_DMP = 0; 
/* [2026-06-12] HMI debug refresh sends two blocking strings. Defer it during
 * TURN_ABS/post-turn gyro lock instead of stealing a random AHDH control frame.
 */
HMI_Update_Debug_Data(g_Vision_Offset, (yaw - g_Yaw_Offset));
}

        // -----------------------------------------------------------
        // [Level 1]: 10ms 硬实时运算层 (100Hz)
        // -----------------------------------------------------------
        if (g_Flag_10ms == true) {
            g_Flag_10ms = false;

            // DMP 读取 + yaw 解卷绕 (消除 atan2 ±180° 跳变)
            {
                float raw_yaw;
                /* [2026-06-12 D项节拍修复]
                 * 主层任务多于调试层，若主循环偶发慢过一个10ms周期，布尔g_Flag_10ms会丢失
                 * “跨过了几个周期”的信息。角速度必须按两次成功DMP读取之间的实际10ms拍数计算，
                 * 否则 diff/0.01f 会把D项阻尼随机放大，表现为同参数下概率性回摆。
                 */
                static uint32_t s_dmp_tick_seq_last = 0;
                static bool s_dmp_tick_seq_ready = false;
                uint32_t dmp_tick_seq_now = g_Control_Tick_Seq;
                uint32_t dmp_elapsed_ticks = 1;
                if (s_dmp_tick_seq_ready) {
                    dmp_elapsed_ticks = dmp_tick_seq_now - s_dmp_tick_seq_last;
                    if (dmp_elapsed_ticks == 0 || dmp_elapsed_ticks > 10) {
                        dmp_elapsed_ticks = 1;
                    }
                }
                float dmp_dt_s = 0.01f * (float)dmp_elapsed_ticks;

                if (mpu_dmp_get_data(&pitch, &roll, &raw_yaw) == 0) {
                    s_dmp_tick_seq_last = dmp_tick_seq_now;
                    s_dmp_tick_seq_ready = true;
                    g_DMP_Elapsed_Ticks = dmp_elapsed_ticks;
                    g_Raw_Yaw = raw_yaw; /* [2026-06-10 诊断] 保存原始值供遥测 */
                    if (s_yaw_first) {
                        s_yaw_first = false;
                        yaw = raw_yaw;
                        /* 【RST修复】: 在解卷绕状态机首次读取DMP时, 同步捕获偏置
                         * 使当前物理航向成为新的逻辑零点, 消除坐标系统错位导致
                         * 的180°整数倍偏移 (旧偏置是解卷绕角度, 新yaw是DMP原生值) */
                        g_Yaw_Offset = raw_yaw;
                        g_Gyro_Z_Rate = 0.0f;
                        s_prev_raw_yaw = raw_yaw;
                    } else {
                        float diff = raw_yaw - s_prev_raw_yaw;
                        /* 解卷绕: 处理 DMP 原生角度 ±180° 边界跳变 */
                        if (diff > 180.0f)      diff -= 360.0f;
                        else if (diff < -180.0f) diff += 360.0f;
                        /* ================================================
                         * [2026-06-12] 野值剔除 v5:
                         * 调试层原始标准为 50°/10ms。主层若偶发跨拍, 固定50°会误杀合法转角；
                         * 固定180°又会放过单拍I2C/DMP野值。这里按实际跨过的10ms拍数放宽阈值:
                         *   1拍=50°, 2拍=100°, 3拍=150°, 最大不超过180°。
                         * 这样既继承调试层AH/DH的抗野值能力, 又兼容主层负载导致的偶发跨拍。
                         */
                        static float s_last_valid_diff = 0.0f;
                        float spike_limit = 50.0f * (float)dmp_elapsed_ticks;
                        if (spike_limit > 180.0f) {
                            spike_limit = 180.0f;
                        }
                        if (fabs(diff) > spike_limit) {
                            yaw += s_last_valid_diff;
                            g_Gyro_Z_Rate = s_last_valid_diff / dmp_dt_s;
                            s_prev_raw_yaw = raw_yaw; /* 复位基准防锁死 */
                        } else {
                            s_last_valid_diff = diff;
                            yaw += diff;
                            g_Gyro_Z_Rate = diff / dmp_dt_s;
                            s_prev_raw_yaw = raw_yaw;
                        }
                    }
                }
            }

            // 核心运动学与传感器融合
            //Update_Chassis_Speed(); //已经在systick里面
            FSM_Update_10ms();
            /* [2026-05-28] 视觉航向校正: FORWARD_TRACK 下视觉确认直行时,
             * 用当前陀螺航向覆写目标航向, 消除行驶中 DMP 漂移累积
             * 置于 FSM_Update_10ms 之后, 与状态机解耦 */
            FSM_Vision_Calibrate_Heading();
            VOFA_Tx_Pump();

            // -------------------------------------------------------
            // [Level 2]: 50ms 遥测分频层 (20Hz)
            // -------------------------------------------------------
            if (++tick_50ms >= 5) {
                tick_50ms = 0;
                ///*
                VOFA_Send_Telemetry(g_FSM_Base_Speed,
                                    FSM_Get_Target_Yaw(),
                                    (yaw - g_Yaw_Offset),
                                    FSM_Get_Yaw_Error(),
                                    /* [2026-05-31] MOTOR+ENC极性已标定, 直接使用原始值, 不再乘极性因子 */
                                    ((float)g_Target_Pulse_L * PULSE_TO_MS_COEF),
                                    ((float)g_Chassis.speed_L * PULSE_TO_MS_COEF),
                                    ((float)g_Target_Pulse_R * PULSE_TO_MS_COEF),
                                    ((float)g_Chassis.speed_R * PULSE_TO_MS_COEF),
                                    target_f,
                                    current_f,
                                    s_current_state,
                                    (char)g_Pi_Cmd);
                //*/
                /*
                if(g_Flag_Send_Trace) { g_Flag_Send_Trace=false; 
        for (int i = 0; i < 128; i++) {
            if (g_Cmd_Trace_Buffer[i]!=0){
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, g_Cmd_Trace_Buffer[i]);
            }else{
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            }
                }
                 }
                 */
                                 /*

    if(g_Pi_Cmd!=last_g_Pi_Cmd){
        last_g_Pi_Cmd=g_Pi_Cmd;
        for (int i = 0; i < 32; i++) {
            if (hst[i]!=0){
                if(hst[i]!=hst[i-1]){
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, hst[i]);
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, ',');
                }
            }else{
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            break;}

    }

    }
    */
        
                if (sendst) {
                    sendst=0;
                    // 扩容至 128 字节，应对 8 个浮点数的格式化字符串
    char Tx_buf[128];
    
    // 
    int len = snprintf(Tx_buf, sizeof(Tx_buf),
     "System State: %s\n",SystemStateStr[s_current_state]);
    
    if (len > 0 && len < sizeof(Tx_buf)) {
        for (int i = 0; i < len; i++) {
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, Tx_buf[i]);
        }
    }

                }
                 if (history) { 
                    history=0;      
        for (int i = 0; i < 32; i++) {
            if (hst[i]!=0){
                if(hst[i]!=hst[i-1]){
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, hst[i]);
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, ',');
                }
            }else{
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            break;}

    }

                    /*            
        for (int i = 0; i < strlen(g_Cmd_Trace_Buffer); i++) {
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, g_Cmd_Trace_Buffer[i]);
        }
*/
                }
            }
            
            // -------------------------------------------------------
            // [Level 3]: 300ms 屏幕刷新分频层 (~3.3Hz)
            // -------------------------------------------------------
            ///*
            if (++tick_300ms >= 20) {
                tick_300ms = 0;
                g_Flag_Read_DMP = 1; // 通知主循环读取 DMP 数据并更新调试显示

                // 2. 静态状态文本 (低频变动，采用缓存比对，仅跳变时发送)
                SystemState_t current_state = FSM_Get_State();
                /* [2026-06-12] HMI UART is blocking. Skip screen text updates
                 * while turning and during the 500ms post-turn gyro lock, so a
                 * random 300ms refresh cannot steal the AHDH control frame.
                 */
                if (!FSM_Is_Turn_Critical_Window()) {
                    if (current_state != last_hmi_state) {
                        HMI_Update_UI_State(current_state);
                        last_hmi_state = current_state;
                    }

                // 3. 目标病房显示 (由视觉组赋给 g_Target_Room)
                    if (g_Target_Room != last_hmi_room) {
                        HMI_Update_Target_Room(g_Target_Room);
                        last_hmi_room = g_Target_Room;
                    }
                }
            }
            //*/
        }
    }
    
}
