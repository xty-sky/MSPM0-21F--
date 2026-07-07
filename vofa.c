// vofa.c (全量代码，请完全覆盖原文件)

#include "vofa.h"
#include "pid.h"
#include "fsm.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "bsp_mpu6050.h"
#include "vision.h"
#include "mymain.h"
#include "hmi.h"
bool sendst2=0;
// 静态缓冲区与标志位 (对外部隐藏)
//static 
uint8_t  rx_buffer[VOFA_RX_MAX_LEN];
uint16_t rx_index = 0;
volatile bool rx_frame_ready = false;
bool sendst=0;
bool history=0;
static char s_vofa_tx_buf[240];
static uint16_t s_vofa_tx_len = 0;
static uint16_t s_vofa_tx_pos = 0;
// 默认基准测试速度 (初始为  m/s)
float g_Target_Velocity_m_s = 0.3f; 
void rx_buffer_clear(void) {
    rx_index = 0;
    rx_frame_ready = false;
    memset(rx_buffer, 0, sizeof(rx_buffer)); // 用memset更高效
}
/* ====================================================================
 * 操作算子：单字节接收压栈 (嵌入 UART_BLE 中断)
 * 物理映射：完美适配 VOFA+ FireWater 下发的 "\n" 或 "\r\n" 帧尾
 * ==================================================================== */
void VOFA_Receive_Byte_ISR(uint8_t rx_data) {
    // 1. 过滤回车符 '\r' (0x0D)，防止其干扰后续的 atof 字符串转换
    if (rx_data == '\r') {
        return; 
    }

    // 2. 预留 1 个字节给 '\0' 结束符
    if (rx_index < (VOFA_RX_MAX_LEN - 1)) {
        // 识别到换行符 '\n' (0x0A)，认定为一帧指令结束
        if (rx_data == '\n') {
            rx_buffer[rx_index] = '\0'; // 替换为 C 语言字符串结束符
            rx_frame_ready = true;      // 举起解析旗帜
        } else {
            // 常规字符，压入栈中
            rx_buffer[rx_index++] = rx_data;
        }
    } else {
        // 缓冲区溢出保护：强行清空复位
        rx_index = 0;
    }
}

void VOFA_Tx_Pump(void) {
    while (s_vofa_tx_pos < s_vofa_tx_len) {
        if (!DL_UART_Main_transmitDataCheck(UART_BLE_INST,
                                            (uint8_t)s_vofa_tx_buf[s_vofa_tx_pos])) {
            break;
        }
        s_vofa_tx_pos++;
    }

    if (s_vofa_tx_pos >= s_vofa_tx_len) {
        s_vofa_tx_len = 0;
        s_vofa_tx_pos = 0;
    }
}

/* ====================================================================
 * 操作算子：FireWater 字符串指令解码树 (主循环异步处理)
 * 物理映射：解析 P1=1.2 格式，并直接写入 PID 结构体内存
 * ==================================================================== */
void VOFA_Parse_Command(void) {
    if (!rx_frame_ready) return;

// --- 1. 内环参数 (解耦左右轮) ---
    if (strncmp((char*)rx_buffer, "P1=", 3) == 0) 
    {pid_speed_L.Kp = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "I1=", 3) == 0) 
    {pid_speed_L.Ki = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }    
    else if (strncmp((char*)rx_buffer, "P3=", 3) == 0) 
    {pid_speed_R.Kp = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "I3=", 3) == 0) 
    {pid_speed_R.Ki = atof((char*)(rx_buffer + 3));    
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
// --- 2. 外环 视觉 PD 参数 (PV, DV) ---
    else if (strncmp((char*)rx_buffer, "PV=", 3) == 0) {
        pid_vision.Kp = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "DV=", 3) == 0) {
        pid_vision.Kd = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    
    // --- 3. 外环 陀螺仪 PD 参数 (PG, DG) ---
    else if (strncmp((char*)rx_buffer, "PG=", 3) == 0) {
        pid_angle_Z.Kp = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "DG=", 3) == 0) {
        pid_angle_Z.Kd = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // --- 3b. 反向循迹独立陀螺仪 PD 参数 (PRG, DRG) ---
    else if (strncmp((char*)rx_buffer, "PRG=", 4) == 0) {
        /* 反向循迹陀螺仪比例系数, 装载后立即生效 */
        g_Rev_Gyro_Kp = atof((char*)(rx_buffer + 4));
        FSM_Apply_Reverse_Gyro_PID(); /* 重新装载至 pid_angle_Z_rev */
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "DRG=", 4) == 0) {
        /* 反向循迹陀螺仪微分系数, 装载后立即生效 */
        g_Rev_Gyro_Kd = atof((char*)(rx_buffer + 4));
        FSM_Apply_Reverse_Gyro_PID(); /* 重新装载至 pid_angle_Z_rev */
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // --- 4. 融合权重调节 (WVF, WVR, WGF, WGR) ---
    else if (strncmp((char*)rx_buffer, "WVF=", 4) == 0) {
        g_Weight_Vision_Fwd = atof((char*)(rx_buffer + 4));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "WVR=", 4) == 0) {
        g_Weight_Vision_Rev = atof((char*)(rx_buffer + 4));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "WGF=", 4) == 0) {
        g_Weight_Gyro_Fwd = atof((char*)(rx_buffer + 4));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "WGR=", 4) == 0) {
        g_Weight_Gyro_Rev = atof((char*)(rx_buffer + 4));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "ALP=", 4) == 0) {
        alp = atof((char*)(rx_buffer + 4));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    
    // 状态机物理参数：转向容差
    else if (strncmp((char*)rx_buffer, "TT=", 3) == 0) {
        g_FSM_Turn_Tolerance = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    // 正向过渡延迟
    else if (strncmp((char*)rx_buffer, "DF=", 3) == 0) {
        g_FSM_Trans_Fwd_Delay = (uint16_t)atoi((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    // 倒退过渡延迟
    else if (strncmp((char*)rx_buffer, "DR=", 3) == 0) {
        g_FSM_Trans_Rev_Delay = (uint16_t)atoi((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    // 倒退过渡时的正向微调速度
    else if (strncmp((char*)rx_buffer, "SR=", 3) == 0) {
        g_FSM_Trans_Rev_Speed = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
// --- 5. 速度测试与紧急复位 ---
    else if (strncmp((char*)rx_buffer, "TF=", 3) == 0) {
        g_FSM_Base_Speed = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "TR=", 3) == 0) {
        g_FSM_Reverse_Speed = atof((char*)(rx_buffer + 3));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "CLEAR", 5) == 0) {
        PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
        PID_Clear(&pid_angle_Z); PID_Clear(&pid_vision);
        //g_FSM_Base_Speed = 0.0f;
        FSM_Init();
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }//已经被RST完全替代
    else if (strncmp((char*)rx_buffer, "RST", 3) == 0) {
        // 1. 强制陀螺仪当前指向为绝对北极 (0度)
        /* 【RST修复】: 不再在此处捕获 g_Yaw_Offset = yaw
         * 偏置捕获已移至 mymain.c DMP 解卷绕的 s_yaw_first 分支
         * 在 DMP 首次读取时同步进行, 确保偏置与新 yaw 在同一坐标系 */
        //g_Vision_Offset = 0;
        // 2. 状态机彻底清空回归待机
        FSM_Init();
        // 3. PID 积分切断
        PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
        PID_Clear(&pid_angle_Z); PID_Clear(&pid_vision);
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strcmp((char*)rx_buffer, "G") == 0) {
        g_Pi_Cmd = 'G';
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strcmp((char*)rx_buffer, "S") == 0) {
        g_Pi_Cmd = 'S';
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strcmp((char*)rx_buffer, "A") == 0) {
        g_Pi_Cmd = 'A';
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strcmp((char*)rx_buffer, "D") == 0) {
        g_Pi_Cmd = 'D';
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strcmp((char*)rx_buffer, "W") == 0) {
        g_Pi_Cmd = 'W';
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strcmp((char*)rx_buffer, "P") == 0) {
        g_Pi_Cmd = 'P';
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    else if (strncmp((char*)rx_buffer, "psd=", 4) == 0) {
            g_FSM_Pre_Stop_Delay  = atof((char*)(rx_buffer + 4));
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
}
    else if (strncmp((char*)rx_buffer, "state", 5) == 0) {
        sendst=1;
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
}
    else if (strncmp((char*)rx_buffer, "his", 3) == 0) {
        history=1;
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '6');
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
}
    /* ==================================================================== */
    
    rx_buffer_clear();
}

/* ====================================================================
 * 操作算子：FireWater 全景波形遥测上报 (21通道)
 * 末尾三通道: DMP跨过的10ms拍数 / DMP FIFO本次读出包数 / pid_angle_Z实际输出。
 * ==================================================================== */
void VOFA_Send_Telemetry(float base_v, float target_yaw, float actual_yaw, float yaw_error,
                         float target_L, float actual_L, float target_R, float actual_R,
                         float target_v, float current_v, SystemState_t state, char cmd) {
    /* 23 channels: ch23 is HMI task state; 0 means not IDLE, 1/2/3 mean IDLE after T1/T2/T3. */
    /* [2026-06-12] Main-layer telemetry must not block the 10ms turn loop.
     * 21 CSV channels at 115200 baud can occupy a whole control frame if sent
     * byte-by-byte with transmitDataBlocking(). Stage one frame and let
     * VOFA_Tx_Pump() feed only currently available UART TX FIFO slots.
     */
    if (s_vofa_tx_len != 0) {
        VOFA_Tx_Pump();
        return;
    }

    int len = snprintf(s_vofa_tx_buf, sizeof(s_vofa_tx_buf),
      "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,"
      "%.3f,%.3f,%.3f,%.3f,%.0f,%.0f,"
      "%.2f,%.2f,%.0f,%.0f,%.2f,%.1f\n",
                       base_v, target_yaw, actual_yaw, yaw_error,
                       target_L, actual_L, target_R, actual_R,
                        out_L, out_R, g_Turn_Entry_Drift,
                       pid_vision.Kp, pid_vision.Kd,
                       pid_angle_Z.Kp, pid_angle_Z.Kd,
                       (float)state, (float)(int)cmd,
                       (float)g_Raw_Yaw, (float)g_Gyro_Z_Rate,
                       (float)g_DMP_Elapsed_Ticks, (float)g_DMP_Drain_Count,
                       pid_angle_Z.out,
                       (FSM_Get_State() == STATE_IDLE) ? (float)g_HMI_Task_State : 0.0f
                        );

    if (len > 0 && len < sizeof(s_vofa_tx_buf)) {
        s_vofa_tx_len = (uint16_t)len;
        s_vofa_tx_pos = 0;
        VOFA_Tx_Pump();
    }
}
