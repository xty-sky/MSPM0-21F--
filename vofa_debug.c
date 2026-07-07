// vofa_debug.c — 调试专用指令解析与遥测
// 接收缓冲区复用 vofa.c 的 rx_buffer/rx_index/rx_frame_ready
// (UART ISR 硬编码调用 VOFA_Receive_Byte_ISR → 写入 vofa.c 缓冲区)

#include "vofa.h"
#include "pid.h"
#include "fsm.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "bsp_mpu6050.h"
#include "mydrive21E.h"

volatile bool g_Flag_Send_Telemetry_Debug = false;

/* ====================================================================
 * 指令解析 (从共享 rx_buffer 读取)
 * ==================================================================== */
void VOFA_Parse_Command_Debug(void) {
    if (!rx_frame_ready) return;

    // ====================================================
    // 1. 状态切换指令
    // ====================================================
    // G/GO → 正向直行
    if (strcmp((char*)rx_buffer, "G") == 0 ||
        strcmp((char*)rx_buffer, "GO") == 0) {
        g_Track_Direction = 1.0f;
        FSM_Debug_Command('G');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'G');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // D/TR → 右转 (精确匹配, 避免误吞 DG=)
    else if (strcmp((char*)rx_buffer, "D") == 0 ||
             strcmp((char*)rx_buffer, "TR") == 0) {
        FSM_Debug_Command('D');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'D');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // A/TL → 左转 (精确匹配, 避免误吞 AH)
    else if (strcmp((char*)rx_buffer, "A") == 0 ||
             strcmp((char*)rx_buffer, "TL") == 0) {
        FSM_Debug_Command('A');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'A');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    /* [2026-05-31 新增] B → 原地180°掉头 (陀螺仪PD大角度转弯测试)
     * 与主构建 fsm.c 中药房180°转弯逻辑对应, 用于在线整定 pid_angle_Z */
    else if (strcmp((char*)rx_buffer, "B") == 0) {
        FSM_Debug_Command('B');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'B');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // S/STOP → 停止
    else if (strcmp((char*)rx_buffer, "S") == 0 ||
             strcmp((char*)rx_buffer, "STOP") == 0) {
        FSM_Debug_Command('S');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'S');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // H → 角度锁定校准
    else if (strcmp((char*)rx_buffer, "H") == 0) {
        FSM_Enter_Angle_Hold();
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'H');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // DH → 右转90°后保持
    else if (strcmp((char*)rx_buffer, "DH") == 0) {
        FSM_Trigger_Turn_Hold_Debug(STATE_TURN_HOLD, -90.0f);
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'D');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'H');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // AH → 左转90°后保持 (必须在 A 后面, 用 strcmp 精确避免冲突)
    else if (strcmp((char*)rx_buffer, "AH") == 0) {
        FSM_Trigger_Turn_Hold_Debug(STATE_TURN_HOLD, 90.0f);
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'A');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'H');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // ====================================================
    // 2. 陀螺仪 PD 参数调整
    // ====================================================
    else if (strncmp((char*)rx_buffer, "PG=", 3) == 0) {
        pid_angle_Z.Kp = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'P');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'G');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    else if (strncmp((char*)rx_buffer, "DG=", 3) == 0) {
        pid_angle_Z.Kd = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'D');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'G');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // ====================================================
    // 2b. 视觉 PD 参数 + 融合权重
    // ====================================================
    else if (strncmp((char*)rx_buffer, "PV=", 3) == 0) {
        pid_vision.Kp = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'P');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'V');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "DV=", 3) == 0) {
        pid_vision.Kd = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'D');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'V');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "WVF=", 4) == 0) {
        g_Weight_Vision_Fwd = atof((char*)(rx_buffer + 3));
        g_Weight_Gyro_Fwd   = 1.0f - g_Weight_Vision_Fwd;  // 联动: 和恒为1
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'W');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'V');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "WGF=", 4) == 0) {
        g_Weight_Gyro_Fwd   = atof((char*)(rx_buffer + 3));
        g_Weight_Vision_Fwd = 1.0f - g_Weight_Gyro_Fwd;    // 联动: 和恒为1
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'W');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'G');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    // 反向权重 (WV= / WG= 只影响正向, 反向独立)
    else if (strncmp((char*)rx_buffer, "WVR=", 4) == 0) {
        g_Weight_Vision_Rev = atof((char*)(rx_buffer + 4));
        g_Weight_Gyro_Rev   = 1.0f - g_Weight_Vision_Rev;  // 联动
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'W');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'R');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "WGR=", 4) == 0) {
        g_Weight_Gyro_Rev   = atof((char*)(rx_buffer + 4));
        g_Weight_Vision_Rev = 1.0f - g_Weight_Gyro_Rev;    // 联动
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'W');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'G');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    // ====================================================
    // 快捷模态切换 (正向)
    // ====================================================
    // VF: 正向纯视觉 (WV=1, WG=0)
    else if (strcmp((char*)rx_buffer, "VF") == 0) {
        g_Track_Direction   = 1.0f;
        g_Weight_Vision_Fwd = 1.0f;
        g_Weight_Gyro_Fwd   = 0.0f;
        s_current_state_debug = STATE_FORWARD_TRACK;
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'V');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'F');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    // GF: 正向纯陀螺 (WV=0, WG=1)
    else if (strcmp((char*)rx_buffer, "GF") == 0) {
        g_Track_Direction   = 1.0f;
        g_Weight_Vision_Fwd = 0.0f;
        g_Weight_Gyro_Fwd   = 1.0f;
        s_current_state_debug = STATE_FORWARD_TRACK;
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'G');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'F');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    // FF: 正向融合 (使用当前 WV/WG)
    else if (strcmp((char*)rx_buffer, "FF") == 0) {
        g_Track_Direction   = 1.0f;
        s_current_state_debug = STATE_FORWARD_TRACK;
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'F');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'F');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // ====================================================
    // 快捷模态切换 (反向)
    // ====================================================
    // VR: 反向纯视觉 (WVR=1, WGR=0)
    else if (strcmp((char*)rx_buffer, "VR") == 0) {
        g_Track_Direction    = -1.0f;
        g_Weight_Vision_Rev  = 1.0f;
        g_Weight_Gyro_Rev    = 0.0f;
        s_current_state_debug = STATE_FORWARD_TRACK;
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'V');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'R');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    // GR: 反向纯陀螺 (WVR=0, WGR=1)
    else if (strcmp((char*)rx_buffer, "GR") == 0) {
        g_Track_Direction    = -1.0f;
        g_Weight_Vision_Rev  = 0.0f;
        g_Weight_Gyro_Rev    = 1.0f;
        s_current_state_debug = STATE_FORWARD_TRACK;
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'G');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'R');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    // FR: 反向融合 (使用当前 WVR/WGR)
    else if (strcmp((char*)rx_buffer, "FR") == 0) {
        g_Track_Direction    = -1.0f;
        s_current_state_debug = STATE_FORWARD_TRACK;
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'F');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'R');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // ====================================================
    // 3. 内环速度 PI 参数调整 (左轮)
    // ====================================================
    else if (strncmp((char*)rx_buffer, "P1=", 3) == 0) {
        pid_speed_L.Kp = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'P');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '1');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "I1=", 3) == 0) {
        pid_speed_L.Ki = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'I');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '1');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // ====================================================
    // 4. 内环速度 PI 参数调整 (右轮)
    // ====================================================
    else if (strncmp((char*)rx_buffer, "P3=", 3) == 0) {
        pid_speed_R.Kp = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'P');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '3');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "I3=", 3) == 0) {
        pid_speed_R.Ki = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'I');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '3');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // ====================================================
    // 5. 基础调参
    // ====================================================
    else if (strncmp((char*)rx_buffer, "TF=", 3) == 0) {
        g_FSM_Base_Speed_Debug = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'T');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'F');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "TR=", 3) == 0) {
        g_FSM_Reverse_Speed_Debug = atof((char*)(rx_buffer + 3));  // 应为负值
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'T');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'R');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "TT=", 3) == 0) {
        g_FSM_Turn_Tolerance_Debug = atof((char*)(rx_buffer + 3));
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'T');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'T');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // ====================================================
    // 6. 系统指令
    // ====================================================
    else if (strncmp((char*)rx_buffer, "RST", 3) == 0) {
        /* 【RST修复】: 偏置捕获移至 mymain4.c s_yaw_first 分支, 此处不再冗余设置 */
        FSM_Init_Debug();
        PID_Clear(&pid_speed_L);
        PID_Clear(&pid_speed_R);
        PID_Clear(&pid_angle_Z);
        PID_Clear(&pid_vision);
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'R');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'S');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'T');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "CLR", 3) == 0) {
        PID_Clear(&pid_speed_L);
        PID_Clear(&pid_speed_R);
        PID_Clear(&pid_angle_Z);
        PID_Clear(&pid_vision);
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'C');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'L');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'R');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "TEL", 3) == 0) {
        g_Flag_Send_Telemetry_Debug = true;
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'T');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'E');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'L');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }
    else if (strncmp((char*)rx_buffer, "NOTEL", 5) == 0) {
        g_Flag_Send_Telemetry_Debug = false;
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, 'N');
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, '\n');
    }

    // ====================================================
    // 7. 陀螺仪性能测试模式
    // ====================================================
    else if (strncmp((char*)rx_buffer, "GT=", 3) == 0) {
        float target_angle = atof((char*)(rx_buffer + 3));
        FSM_Enter_Gyro_Tune_Mode(target_angle);
        char tx_buf[64];
        int len = snprintf(tx_buf, sizeof(tx_buf),
                          "GYRO_TUNE_START: target=%.1f\n", target_angle);
        for (int i = 0; i < len; i++) {
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, tx_buf[i]);
        }
    }
    else if (strncmp((char*)rx_buffer, "GTSET=", 6) == 0) {
        float new_target = atof((char*)(rx_buffer + 6));
        FSM_Set_Gyro_Target(new_target);
        char tx_buf[64];
        int len = snprintf(tx_buf, sizeof(tx_buf),
                          "GYRO_TARGET_UPDATE: %.1f\n", new_target);
        for (int i = 0; i < len; i++) {
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, tx_buf[i]);
        }
    }
    else if (strncmp((char*)rx_buffer, "GT_EXIT", 7) == 0 ||
             strncmp((char*)rx_buffer, "GTEXIT", 6) == 0) {
        FSM_Exit_Gyro_Tune_Mode();
        char tx_buf[32];
        int len = snprintf(tx_buf, sizeof(tx_buf), "GYRO_TUNE_EXIT\n");
        for (int i = 0; i < len; i++) {
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, tx_buf[i]);
        }
    }

    rx_buffer_clear();
}

/* ====================================================================
 * 遥测发送 (CSV 全浮点, VOFA+ FireWater 协议)
 * ==================================================================== */
void VOFA_Send_Telemetry_Debug(float base_v, float target_yaw, float actual_yaw, float yaw_error,
                               float vis_Kp, float vis_Kd,
                               float target_L, float actual_L, float target_R, float actual_R,
                               float vis_tgt, float vis_act,
                               float vis_offset, float vis_updated,
                               SystemState_t state, char cmd) {

    if (!g_Flag_Send_Telemetry_Debug) return;

    char tx_buf[256];
    int len = snprintf(tx_buf, sizeof(tx_buf),
                       "%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.0f\n",
                       base_v,
                       target_yaw,
                       actual_yaw,
                       yaw_error,
                       pid_angle_Z.Kp,
                       pid_angle_Z.Kd,
                       vis_Kp,
                       vis_Kd,
                       target_L,
                       actual_L,
                       target_R,
                       actual_R,
                       vis_tgt,
                       vis_act,
                       vis_offset,
                       vis_updated,
                       (float)state);

    if (len > 0 && len < sizeof(tx_buf)) {
        for (int i = 0; i < len; i++) {
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, tx_buf[i]);
        }
    }
}

/* ====================================================================
 * PID 参数调试打印
 * ==================================================================== */
void VOFA_Debug_Print_PID_Params(void) {
    char tx_buf[256];
    int len = snprintf(tx_buf, sizeof(tx_buf),
        "[PID] Gyro: Kp=%.3f, Kd=%.3f | Speed_L: Kp=%.1f, Ki=%.1f | Speed_R: Kp=%.1f, Ki=%.1f\n",
        pid_angle_Z.Kp, pid_angle_Z.Kd,
        pid_speed_L.Kp, pid_speed_L.Ki,
        pid_speed_R.Kp, pid_speed_R.Ki);

    if (len > 0 && len < sizeof(tx_buf)) {
        for (int i = 0; i < len; i++) {
            DL_UART_Main_transmitDataBlocking(UART_BLE_INST, tx_buf[i]);
        }
    }
}
