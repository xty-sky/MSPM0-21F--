#ifndef COOP_H
#define COOP_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>

/* ====================================================================
 * 跨文件并发通信标志位 (声明)
 * 作用：由 coop.c 的 UART 中断拉高，由 fsm.c 的状态机消费并清零
 * 约束：必须使用 volatile 防止编译器寄存器缓存优化导致死锁
 * ==================================================================== */
extern volatile bool g_Coop_Remote_Go_Flag;

/* ====================================================================
 * 核心对外算子
 * ==================================================================== */
// 发送算子：车 1 抵达目标后调用，向车 2 发送唤醒及目标房号
void Coop_Send_Start_Cmd(uint8_t room_num);

// 接收算子：挂载于 UART_COOP_INST 中断，执行 O(1) 状态机解析
void Coop_Receive_Byte_ISR(uint8_t rx_data);

#endif // COOP_H