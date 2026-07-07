// mydrive21E.c
#include "mydrive21E.h"
#include "vofa.h"
#include "vision.h"
#include "fsm.h"
#include "hmi.h"
#include "coop.h"
#include "ti/driverlib/driverlib.h" // 确保能调用 DL_Common_delayCycles
// 全局变量实体定义 (只在这里定义一次)
// volatile 关键字约束：强制编译器每次都从内存地址中读取该变量，禁止将其优化缓存到 CPU 寄存器中。
// 为什么？因为这些变量会在前台中断（如 SysTick 或 GPIO 中断）中被硬件事件随机修改，后台主循环必须看到最新值。
volatile ChassisState_t g_Chassis = {0};
volatile bool g_Flag_10ms = true;
volatile uint32_t g_Control_Tick_Seq = 0;

// 在文件顶部的全局变量区
AdaptiveKalman_t akf_speed_L,akf_speed_R;
/* ====================================================================
 * 操作单元：单侧电机防直通 PWM 覆写
 * 输入参量：
 *   motor_id: 0 (左轮_TIMA0), 1 (右轮_TIMA1)
 *   pwm_value: 带符号速度期望值 (-1000 ~ 1000)
 * ==================================================================== */
void Motor_SetPWM(uint8_t motor_id, int16_t pwm_value) {
    // 1. 物理极性软件解耦
    if (motor_id == 0) pwm_value *= MOTOR_L_POLARITY;
    else               pwm_value *= MOTOR_R_POLARITY;

    // 2. 输入算子限幅
    if (pwm_value > PWM_PERIOD_MAX)  pwm_value = PWM_PERIOD_MAX;
    if (pwm_value < -PWM_PERIOD_MAX) pwm_value = -PWM_PERIOD_MAX;

    // 3. 占空比映射 (摒弃画蛇添足的倒置运算)
    uint32_t cc_active = (uint32_t)(pwm_value > 0 ? pwm_value : -pwm_value);
    uint32_t cc_coast  = 0;               // 0% 占空比 -> 持续低电平 (配合单臂驱动)
    // [2026-05-31 致命修复] 刹车CC=0, 见下方Motor_SetPWM(pwm=0)分支注释
    // [废弃] cc_brake=PWM_PERIOD_MAX-1=999 和 cc_brake=getLoadValue()=1000
    //   两者在EDGE_ALIGN递减计数模式下都产生0%占空 → COAST滑行 → 刹车完全失效!

    // 4. 硬件通道路由
    if (motor_id == 0) { 
        // 左轮驱动网络
        if (pwm_value > 0) {
            DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, cc_active, GPIO_PWM_LEFT_C2_IDX);
            DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, cc_coast,  GPIO_PWM_LEFT_C3_IDX);
        } else if (pwm_value < 0) {
            DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, cc_coast,  GPIO_PWM_LEFT_C2_IDX);
            DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, cc_active, GPIO_PWM_LEFT_C3_IDX);
        } else {
            /* [2026-05-31 致命修复] 左轮刹车: CC=0 → ≈100%占空比 → 短路制动
             * 根因: EDGE_ALIGN递减计数(LOAD→0), LACT=HIGH, CDACT=LOW
             * 旧CC=period(1000): CDACT在counter=LOAD瞬间覆盖LACT → 0%占空比→ COAST
             * 新CC=0: CDACT仅在counter=0触发 → ≈100%占空比 → 绕组短路制动
             * 旧代码[2026-05-25]: CC=period_L → 同样是0%占空 → 刹车完全失效!
             * 旧代码[v1]: CC=999 → 同样是0%占空 → 刹车完全失效! */
            // [旧 2026-05-25] uint32_t period_L = DL_TimerA_getLoadValue(PWM_LEFT_INST);
            // [旧 2026-05-25] DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, period_L, ...);
            DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, 0, GPIO_PWM_LEFT_C2_IDX);
            DL_TimerA_setCaptureCompareValue(PWM_LEFT_INST, 0, GPIO_PWM_LEFT_C3_IDX);
        }
    }
    else {
        // 右轮驱动网络 (TIMA1 单定时器对称架构)
        if (pwm_value > 0) {
            DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, cc_active, GPIO_PWM_RIGHT_C0_IDX);
            DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, cc_coast,  GPIO_PWM_RIGHT_C1_IDX);
        } else if (pwm_value < 0) {
            DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, cc_coast,  GPIO_PWM_RIGHT_C0_IDX);
            DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, cc_active, GPIO_PWM_RIGHT_C1_IDX);
        } else {
            /* [2026-05-31 致命修复] 右轮刹车: CC=0 → ≈100%占空比 → 短路制动 (同上) */
            // [旧 2026-05-25] uint32_t period_R = DL_TimerA_getLoadValue(PWM_RIGHT_INST);
            // [旧 2026-05-25] DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, period_R, ...);
            DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, 0, GPIO_PWM_RIGHT_C0_IDX);
            DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, 0, GPIO_PWM_RIGHT_C1_IDX);
        }
    }
}

void AKF_Init(AdaptiveKalman_t *kf, float noise_margin) {
    kf->x = 0.0f;
    kf->P = 1.0f;
    kf->noise_margin = noise_margin;
}

float AKF_Update(AdaptiveKalman_t *kf, float measured_val) {
    // 1. 计算新息 (实际测量值与当前估计值的差)
    float innovation = measured_val - kf->x;
    float abs_inn = fabs(innovation);

    // 2. [核心物理映射] 自适应调整过程噪声 Q
    float Q;
    if (abs_inn <= kf->noise_margin) {
        // 【模态1匀速/模态2寻线平滑期】：完全处于量化噪声底噪内
        // 极度信任预测，抹平毛刺
        Q = 0.005f; 
    } else {
        // 【模态1起步/模态3转弯/模态4倒车】：发生真实物理机动
        // 误差超出底噪的部分，平方级放大 Q，瞬间撕开滤波延迟
        float over_err = abs_inn - kf->noise_margin;
        Q = 0.005f + 0.05f * over_err * over_err; 
        
        // 封顶保护，防止极端飞点(如杜邦线松动瞬间)导致系统崩溃
        if (Q > 2.0f) Q = 2.0f; 
    }

    // 3. 卡尔曼迭代 (固定 R = 1.0f)
    kf->P = kf->P + Q;
    float K = kf->P / (kf->P + 1.0f); 
    kf->x = kf->x + K * innovation;
    kf->P = (1.0f - K) * kf->P; // 自动收敛，不会无限递增

    return kf->x;
}
/* ====================================================================
 * 操作单元：底盘增量速度提取 (供 10ms 时间片调用)
 * ==================================================================== */
//这是没有滤波的速度更新
/*
void Update_Chassis_Speed(void) {
    // 左轮 (软件 EXTI 计数值差分)
    volatile uint16_t current_enc_L =g_Chassis.encoder_L_count;

    volatile int16_t delta_L = (int16_t)(current_enc_L - g_Chassis.last_enc_L);
    g_Chassis.last_enc_L = current_enc_L;
    g_Chassis.speed_L   = (delta_L * ENC_L_POLARITY); 

    // 右轮 (软件 EXTI 计数值差分)
  volatile uint16_t current_enc_R = g_Chassis.encoder_R_count;
  volatile int16_t delta_R = (int16_t)(current_enc_R - g_Chassis.last_enc_R);
    g_Chassis.last_enc_R = current_enc_R;
    g_Chassis.speed_R  = (delta_R * ENC_R_POLARITY);
}
*/

//这是有均值滤波的速度更新
///*
void Update_Chassis_Speed(void) {
    /* [2026-05-25 修复] current_enc / delta 类型由 uint16_t/int16_t → int32_t,
     * 与 encoder_count(int32_t) 和 last_enc(int32_t) 类型统一, 消除静默截断 */
    // 左轮 (软件 EXTI 计数值差分)
    // volatile uint16_t current_enc_L = g_Chassis.encoder_L_count;  /* 旧, 截断高16位 */
    volatile int32_t current_enc_L = g_Chassis.encoder_L_count;
    // volatile int16_t delta_L = (int16_t)(current_enc_L - g_Chassis.last_enc_L);  /* 旧 */
    volatile int32_t delta_L = current_enc_L - g_Chassis.last_enc_L;
    g_Chassis.last_enc_L = current_enc_L;

    // 右轮 (软件 EXTI 计数值差分)
    // volatile uint16_t current_enc_R = g_Chassis.encoder_R_count;  /* 旧, 截断高16位 */
    volatile int32_t current_enc_R = g_Chassis.encoder_R_count;
    // volatile int16_t delta_R = (int16_t)(current_enc_R - g_Chassis.last_enc_R);  /* 旧 */
    volatile int32_t delta_R = current_enc_R - g_Chassis.last_enc_R;
    g_Chassis.last_enc_R = current_enc_R;

    g_Chassis.speed_L = (int16_t)(delta_L * ENC_L_POLARITY);
    g_Chassis.speed_R = (int16_t)(delta_R * ENC_R_POLARITY);
}
//*/

//这是有卡尔曼滤波的速度更新,先注释掉，不使能滤波
/*
void Update_Chassis_Speed(void) {
    volatile uint16_t current_enc_L = g_Chassis.encoder_L_count;
    volatile int16_t delta_L = (int16_t)(current_enc_L - g_Chassis.last_enc_L);
    g_Chassis.last_enc_L = current_enc_L;
    
    // 计算原始裸数据
    float raw_speed_L = (float)(delta_L * ENC_L_POLARITY); 
    // 【注入点】：通过卡尔曼算子清洗后，再赋值给底盘状态
    g_Chassis.speed_L = AKF_Update(&akf_speed_L, raw_speed_L); 

    volatile uint16_t current_enc_R = g_Chassis.encoder_R_count;
    volatile int16_t delta_R = (int16_t)(current_enc_R - g_Chassis.last_enc_R);
    g_Chassis.last_enc_R = current_enc_R;
    
    // 计算原始裸数据
    float raw_speed_R = (float)(delta_R * ENC_R_POLARITY);
    // 【注入点】：通过卡尔曼算子清洗后，再赋值给底盘状态
    g_Chassis.speed_R = AKF_Update(&akf_speed_R, raw_speed_R);
}
*/
/* ====================================================================
 * [前台中断] 1ms 滴答定时器
 * ==================================================================== */
void SysTick_Handler(void) {
    static uint8_t tick_count = 0;
    tick_count++;
    if (tick_count >= 10) { 
        tick_count = 0;
        g_Flag_10ms = true;
        g_Control_Tick_Seq++;
// 【核心绝杀】：在这里调用测速！
        // 硬件级绝对定时，无视主循环任何阻塞，delta_L 将永远等于完美的 100ms 脉冲！突变彻底消失！
        Update_Chassis_Speed(); 
    }
    }
/* ====================================================================
 * [前台中断] GPIO PORTA 汇总响应枢纽 (GROUP0) -> 专供左轮 PA29
 *
 * ==================================================================== */
/*
void GROUP2_IRQHandler(void) {
    uint32_t gpioA_status = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_LEFT_PIN_QEI_L_A_PIN);

    // 响应左轮 PA29 上升沿
    if (gpioA_status & GPIO_LEFT_PIN_QEI_L_A_PIN) { 
        // 读取 PA30 判定方向
        bool phase_L_b = DL_GPIO_readPins(GPIOA, GPIO_LEFT_PIN_QEI_L_B_PIN);
        if (phase_L_b) g_Chassis.encoder_L_count++;
        else           g_Chassis.encoder_L_count--;
        
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_LEFT_PIN_QEI_L_A_PIN);
    }
}

*/
/* ====================================================================
 * [前台中断] GPIO 汇总响应枢纽 (GROUP1) -> 专供 左轮、右轮、载药
 * 物理映射：MSPM0G 系列中，所有 GPIO (A口与B口) 的外部中断，
 * 全部统一被硬件 Event Fabric 路由至 GROUP1_IRQHandler！
 * ==================================================================== */
void GROUP1_IRQHandler(void) {
    uint32_t gpioB_status = DL_GPIO_getEnabledInterruptStatus(GPIOB, 
                                GPIO_SENSOR_PIN_QEI_R_A_PIN | 
                                GPIO_SENSOR_PIN_LOAD_PHOTO_PIN);
    uint32_t gpioA_status = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_LEFT_PIN_QEI_L_A_PIN);

    // 响应左轮 PA29 上升沿
    if (gpioA_status & GPIO_LEFT_PIN_QEI_L_A_PIN) { 
        // 2. 【编码器边沿二次确认】：32MHz 下 320 cycles 约 10us。
        // 原 3200 cycles 实为约 100us，高速脉冲期会拖慢主循环并放大 DMP 跨拍抖动。
        DL_Common_delayCycles(320); 
// 1. 清除中断标志，防止重复进入
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_LEFT_PIN_QEI_L_A_PIN);
        

        // 3. 【边缘二次确权】：既然是上升沿触发，此刻真实的物理电平必须是高电平(1)！
        // 如果读到的是 0，说明刚才触发中断的只是一个噪声尖峰，直接抛弃！
        if (DL_GPIO_readPins(GPIOA, GPIO_LEFT_PIN_QEI_L_A_PIN) != 0) {
            
            // 确权成功！这是一个真实可靠的物理刻度。读取 B 相判定正反转
                    // 读取 PA30 判定方向
        bool phase_L_b = DL_GPIO_readPins(GPIOA, GPIO_LEFT_PIN_QEI_L_B_PIN);
        if (phase_L_b) g_Chassis.encoder_L_count++;
        else           g_Chassis.encoder_L_count--;
        }
    }
    // ==========================================================
    // 右轮 A 相测速响应 (同理)
    // ==========================================================
    if (gpioB_status & GPIO_SENSOR_PIN_QEI_R_A_PIN) { 
        DL_Common_delayCycles(320); // 32MHz 下约 10us，保持左右轮一致

        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_SENSOR_PIN_QEI_R_A_PIN);
        
        
        // 边缘确权
        if (DL_GPIO_readPins(GPIOB, GPIO_SENSOR_PIN_QEI_R_A_PIN) != 0) {
            bool phase_R_b = DL_GPIO_readPins(GPIOB, GPIO_SENSOR_PIN_QEI_R_B_PIN);
            if (phase_R_b) g_Chassis.encoder_R_count++; 
            else           g_Chassis.encoder_R_count--;
        }
    }
        
    /* -----------------------------------------------------------
     * 3. 载药光电对管 (PB19) 下降沿
     * ----------------------------------------------------------- */

     /*
    if (gpioB_status & GPIO_SENSOR_PIN_LOAD_PHOTO_PIN) {
        DL_Common_delayCycles(3200); 
// 1. 清除中断标志，防止重复进入
        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_SENSOR_PIN_LOAD_PHOTO_PIN);
        

        // 3. 【边缘二次确权】：既然是下降沿触发，此刻真实的物理电平必须是00000！
        // 如果读到的是 1，说明刚才触发中断的只是一个噪声尖峰，直接抛弃！
        if (DL_GPIO_readPins(GPIOA, GPIO_SENSOR_PIN_LOAD_PHOTO_PIN) != 1) {


// 【注入点】：触发状态机倒车逻辑
        FSM_Trigger_Reverse_Start();
        DL_GPIO_clearInterruptStatus(GPIOB, GPIO_SENSOR_PIN_LOAD_PHOTO_PIN);
    }
    }*/
}
/* ====================================================================
 * [前台中断] 异常拦截与通信兜底算子 (防 Default_Handler 死锁)
 * 物理映射：当 UART 接收到数据触发中断时，必须读取寄存器以清除中断标志，
 * 否则会无限次反复进入中断导致主程序挂起。
 * ==================================================================== */

// UART3: HMI 串口屏
void UART_HMI_INST_IRQHandler(void) {
    switch (DL_UART_Main_getPendingInterrupt(UART_HMI_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        {
                uint8_t rxData = DL_UART_Main_receiveData(UART_HMI_INST); 
                // 【注入点】：喂入 HMI 状态机
                HMI_Receive_Byte_ISR(rxData);
        }
        break;
        default: break;
    }
}

// UART1: 树莓派
void UART_PI_INST_IRQHandler(void) {
    // 1. 获取当前被使能且触发的中断状态。
    // 第二个参数 0xFFFFFFFF 代表查询所有可能的中断位。
    uint32_t pending_status = DL_UART_Main_getEnabledInterruptStatus(UART_PI_INST, 0xFFFFFFFF);

    // 2. 判断是否是 RX (接收) 中断 (注意：官方掩码宏是 DL_UART_INTERRUPT_RX)
    if (pending_status & DL_UART_INTERRUPT_RX) {
        
        // 3. 必须手动清除 RX 中断标志位
        DL_UART_Main_clearInterruptStatus(UART_PI_INST, DL_UART_INTERRUPT_RX);

        // 4. 【核心灵魂】：榨干 FIFO！只要里面有数据，就一直读出喂给状态机
        while (!DL_UART_Main_isRXFIFOEmpty(UART_PI_INST)) {
            uint8_t rxData = DL_UART_Main_receiveData(UART_PI_INST); 
            Vision_Receive_Byte_ISR(rxData); 
        }
    }
    
    // 5. 【极其关键的防御】：视觉高频数据极易引发硬件 FIFO 溢出(Overrun)
    // 如果发生溢出，必须强行清除溢出标志位，否则单片机会永远卡死在此中断里！
    if (pending_status & DL_UART_INTERRUPT_OVERRUN_ERROR) {
        DL_UART_Main_clearInterruptStatus(UART_PI_INST, DL_UART_INTERRUPT_OVERRUN_ERROR);
    }
}
// UART2: 调参蓝牙

/* [2026-06-01] 原始 ISR 模式 (NVIC 暂不触发, 收数由 mymain4 主循环轮询兜底) */
void UART_BLE_INST_IRQHandler(void) {
    uint32_t pending_status = DL_UART_Main_getEnabledInterruptStatus(UART_BLE_INST, 0xFFFFFFFF);

    if (pending_status & DL_UART_INTERRUPT_RX) {
        DL_UART_Main_clearInterruptStatus(UART_BLE_INST, DL_UART_INTERRUPT_RX);
        while (!DL_UART_Main_isRXFIFOEmpty(UART_BLE_INST)) {
            uint8_t rxData = DL_UART_Main_receiveData(UART_BLE_INST);
            VOFA_Receive_Byte_ISR(rxData);
        }
    }

    if (pending_status & DL_UART_INTERRUPT_OVERRUN_ERROR) {
        DL_UART_Main_clearInterruptStatus(UART_BLE_INST, DL_UART_INTERRUPT_OVERRUN_ERROR);
    }
}
// UART0: 双车协同
// UART0: 双车协同
void UART_COOP_INST_IRQHandler(void) {
    uint32_t pending_status = DL_UART_Main_getEnabledInterruptStatus(UART_COOP_INST, 0xFFFFFFFF);

    if (pending_status & DL_UART_INTERRUPT_RX) {
        DL_UART_Main_clearInterruptStatus(UART_COOP_INST, DL_UART_INTERRUPT_RX);
        while (!DL_UART_Main_isRXFIFOEmpty(UART_COOP_INST)) {
            uint8_t rxData = DL_UART_Main_receiveData(UART_COOP_INST);
            Coop_Receive_Byte_ISR(rxData); // 喂给协同状态机
        }
    }

    if (pending_status & DL_UART_INTERRUPT_OVERRUN_ERROR) {
        DL_UART_Main_clearInterruptStatus(UART_COOP_INST, DL_UART_INTERRUPT_OVERRUN_ERROR);
    }
}
