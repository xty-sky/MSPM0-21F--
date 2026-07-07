#ifndef VISION_H
#define VISION_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"
// 物理记忆重载：假设摄像头分辨率水平宽度为 320，中心点即为 160
#define VISION_CENTER_X 160.0f
extern uint8_t g_Target_Room;
// ====================================================================
// 全局视觉状态暴露 (供 PID 融合环与主状态机调用)
// ====================================================================
extern float g_Vision_Offset;   // 视觉横向偏差值 (像素)。正数代表偏右，负数代表偏左
extern uint8_t g_Pi_Cmd;        // 树莓派下发的顶层动作指令 (如 'S', 'W')
extern bool g_Vision_Updated;   // 数据刷新标志位 (防旧数据滞留)
extern uint8_t vision_rx_idx;
extern uint8_t vision_idle_cnt;
extern float target_f  ;
extern float current_f;

// 在 vision.c 顶部定义全局共享变量 (别忘了在 vofa.h 或 vision.h 加上 extern)
extern char g_Cmd_Trace_Buffer[128];
extern bool g_Flag_Send_Trace;
extern uint8_t s_trace_len;
extern uint8_t s_last_traced_cmd;


extern volatile char s_temp_cmd;
// ====================================================================
// 核心操作算子接口
// ====================================================================
void Vision_Receive_Byte_ISR(uint8_t rx_data);
void Vision_Parse_Frame(void);

#endif // VISION_H