#ifndef MYMAIN_H
#define MYMAIN_H
#include "ti_msp_dl_config.h"
#include "mydrive21E.h"
#include "pid.h"
#include "vofa.h"
#include "vision.h"
#include "fsm.h"
#include "hmi.h"
#include <stdio.h>
#include "bsp_mpu6050.h"
#include "inv_mpu.h"
#include "board.h"

// ====================================================================
// 全局变量声明 (已收敛到各专用头文件: bsp_mpu6050.h / pid.h / fsm.h)
// 此处仅保留跨模块桥接变量
// ====================================================================
extern float g_Vision_Offset;
extern uint8_t g_Target_Room;
extern uint32_t g_DMP_Elapsed_Ticks;
extern uint32_t g_DMP_Drain_Count;

// ====================================================================
// yaw 解卷绕状态复位 (由 mymain.c / mymain4.c 定义, FSM_Init 调用)
// ====================================================================
void Yaw_Integrator_Reset(void);        // 主构建, 复位 mymain.c 静态变量
void Yaw_Integrator_Reset_Debug(void);  // 调试构建, 复位 mymain4.c 静态变量

// ====================================================================
// 主业务逻辑入口
// ====================================================================
void mymain(void);
void mymain2(void);
void mymain3(void);
void mymain4(void);

void mymaintest1(void);

#endif//mymain.h
