#include "mymain.h"
#include "vofa.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "inv_mpu.h"

// pitch/roll/yaw/g_Yaw_Offset/g_Gyro_Z_Rate 定义在 mymain.c + pid.c (主构建)
// 此处仅保留调试构建私有的解卷绕状态

static float s_prev_raw_yaw = 0.0f;
static bool  s_yaw_first = true;

void Yaw_Integrator_Reset_Debug(void) {
    s_yaw_first = true;
    s_prev_raw_yaw = 0.0f;
}

/* ====================================================================
 * [2026-06-01] 调试构建 — 陀螺仪PD整定 + 转弯性能测试
 * 下地调试用, 遥测经 UART_BLE→VOFA+ FireWater 波形
 * 指令集: A/D/B/G/S/H/DH/AH/TEL/NOTEL/PG=/DG=/TF=/TT= ...
 * ==================================================================== */
void mymain4(void)
{
    /* ---- 硬件底座 ---- */
    DL_SYSTICK_enableInterrupt();
    DL_SYSTICK_enable();
    SYSCFG_DL_init();

    NVIC_DisableIRQ(UART_COOP_INST_INT_IRQN);
    DL_UART_Main_clearInterruptStatus(UART_COOP_INST, DL_UART_MAIN_INTERRUPT_RX);

    NVIC_SetPriority(GPIO_SENSOR_INT_IRQN, 0);
    NVIC_SetPriority(GPIO_LEFT_INT_IRQN, 0);
    NVIC_SetPriority(SysTick_IRQn, 1);
    NVIC_SetPriority(UART_BLE_INST_INT_IRQN, 2);
    NVIC_SetPriority(UART_HMI_INST_INT_IRQN, 2);
    NVIC_SetPriority(UART_PI_INST_INT_IRQN, 2);

    /* ---- DMP 初始化 ---- */
    while (MPU6050_Init() != 0) { delay_ms(500); }
    while (mpu_dmp_init() != 0) { delay_ms(500); }

    {
        float raw_yaw_init;
        while (mpu_dmp_get_data(&pitch, &roll, &raw_yaw_init) != 0)
            delay_ms(10);
        yaw = raw_yaw_init;
        s_prev_raw_yaw = raw_yaw_init;
        s_yaw_first = false;
    }

    DL_TimerA_startCounter(PWM_LEFT_INST);
    DL_TimerA_startCounter(PWM_RIGHT_INST);

    NVIC_EnableIRQ(UART_HMI_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_BLE_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_PI_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_LEFT_INT_IRQN);
    NVIC_EnableIRQ(GPIO_SENSOR_INT_IRQN);
    __enable_irq();

    LED_ALL_OFF(); LED_GREEN_ON();
    Motor_SetPWM(0, 0); Motor_SetPWM(1, 0);

    /* ---- PID 初始化 ---- */
    /* [2026-06-01] 极性已标定 (MOTOR=1,ENC正确), 内环从保守Kp=40起步 */
    PID_Init(&pid_speed_L, 40.0f, 1.0f, 0.0f, 900.0f, 800.0f, 0.2f, 150.0f);
    PID_Init(&pid_speed_R, 40.0f, 1.0f, 0.0f, 900.0f, 800.0f, 0.2f, 150.0f);
    PID_Init(&pid_vision, 0.05f, 0.0f, 0.02f, 15.0f, 0.0f, 0.5f, 0.0f);
    PID_Init(&pid_angle_Z, 0.25f, 0.0f, 0.03f, 30.0f, 0.0f, 2.0f, 0.0f);

    FSM_Init_Debug();

    static uint8_t tick_50ms  = 0;
    static uint8_t tick_100ms = 0;

    g_Flag_Send_Telemetry_Debug = true;
    HMI_Send_Raw_String("t_stat.txt=\"ON_THEBOARD\"");

    while (1)
    {
        /* [2026-06-01] UART_BLE 收数: 轮询 RIS 寄存器
         * ISR 因 NVIC 向量表问题暂不触发, 用主循环轮询替代,
         * 与 ISR 内部 getEnabledInterruptStatus 逻辑完全一致 */
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

        VOFA_Parse_Command_Debug();
        HMI_Parse_Command();

        if (g_Flag_10ms == true) {
            g_Flag_10ms = false;

            /* DMP 读取 + yaw 解卷绕 */
            {
                float raw_yaw;
                /* 与主层保持同一角速度估算方式，防止调试/主线因主循环负载不同再次分叉。 */
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
                    if (s_yaw_first) {
                        s_yaw_first = false;
                        yaw = raw_yaw;
                        g_Yaw_Offset = raw_yaw;
                        g_Gyro_Z_Rate = 0.0f;
                        s_prev_raw_yaw = raw_yaw;
                    } else {
                        float diff = raw_yaw - s_prev_raw_yaw;
                        if (diff > 180.0f)      diff -= 360.0f;
                        else if (diff < -180.0f) diff += 360.0f;
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

            FSM_Update_10ms_Debug();

            /* 50ms 遥测 → VOFA+ */
            if (++tick_50ms >= 5) {
                tick_50ms = 0;
                VOFA_Send_Telemetry_Debug(
                    g_FSM_Base_Speed_Debug,
                    FSM_Get_Target_Yaw_Debug(),
                    GET_RELATIVE_YAW(),
                    FSM_Get_Yaw_Error_Debug(),
                    pid_vision.Kp,
                    pid_vision.Kd,
                    ((float)g_Target_Pulse_L * PULSE_TO_MS_COEF),
                    ((float)g_Chassis.speed_L * PULSE_TO_MS_COEF),
                    ((float)g_Target_Pulse_R * PULSE_TO_MS_COEF),
                    ((float)g_Chassis.speed_R * PULSE_TO_MS_COEF),
                    target_f,
                    current_f,
                    g_Vision_Offset,
                    (float)g_Vision_Updated,
                    FSM_Get_State_Debug(),
                    0);
            }

            /* 100ms LED 状态指示 */
            if (++tick_100ms >= 10) {
                tick_100ms = 0;
                SystemState_t st = FSM_Get_State_Debug();
                LED_ALL_OFF();
                if (st == STATE_IDLE)           LED_RED_ON();
                else if (st == STATE_FORWARD_TRACK) LED_GREEN_ON();
                else if (st == STATE_TURN_ABS)  LED_YELLOW_ON();
                else if (st == STATE_ANGLE_HOLD) { LED_RED_ON(); LED_YELLOW_ON(); }
                else if (st == STATE_TURN_HOLD)  { LED_YELLOW_ON(); }
            }
        }
    }
}
