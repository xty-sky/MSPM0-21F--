#ifndef VOFA_H
#define VOFA_H

#include "ti_msp_dl_config.h"
#include "pid.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "fsm.h"

// 蓝牙串口接收缓冲约束
#define VOFA_RX_MAX_LEN 64

// ====================================================================
// 【共享】: 接收缓冲区 (ISR 写入, vofa.c / vofa_debug.c 均读取)
// ====================================================================
extern uint8_t  rx_buffer[VOFA_RX_MAX_LEN];
extern uint16_t rx_index;
extern volatile bool rx_frame_ready;

// ====================================================================
// 【Main 构建】: vofa.c 专属声明
// ====================================================================
extern bool sendst2;
extern float g_Target_Velocity_m_s;
extern bool sendst;
extern bool history;
extern void VOFA_Receive_Byte_ISR(uint8_t rx_data);
extern void VOFA_Parse_Command(void);
extern void VOFA_Tx_Pump(void);
extern void VOFA_Send_Telemetry(float base_v, float target_yaw, float actual_yaw, float yaw_error,
                                 float target_L, float actual_L, float target_R, float actual_R,
                                 float target_v, float current_v, SystemState_t state, char cmd);
extern void rx_buffer_clear(void);

// ====================================================================
// 【Debug 构建】: vofa_debug.c 专属声明 (共享 vofa.c 的接收缓冲区)
// ====================================================================
extern volatile bool g_Flag_Send_Telemetry_Debug;

extern void VOFA_Parse_Command_Debug(void);
extern void VOFA_Send_Telemetry_Debug(float base_v, float target_yaw, float actual_yaw, float yaw_error,
                                       float vis_Kp, float vis_Kd,
                                       float target_L, float actual_L, float target_R, float actual_R,
                                       float vis_tgt, float vis_act,
                                       float vis_offset, float vis_updated,
                                       SystemState_t state, char cmd);
extern void VOFA_Debug_Print_PID_Params(void);

#endif
