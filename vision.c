#include "vision.h"
#include <string.h>

// ====================================================================
// 全局变量实体 (对外暴露，供给 FSM 和 mydrive 消费)
// ====================================================================
float g_Vision_Offset = 0.0f; 
uint8_t g_Pi_Cmd = 0;         
bool g_Vision_Updated = false;
volatile char s_temp_cmd = 0;
// 【新增】：病房号同步目标 (给 HMI 串口屏转发使用)
uint8_t g_Target_Room = 0;    
float target_f = 200 ;
float current_f = 200 ;
/* ====================================================================
 * 2. 状态机私有内存区 (静态隔离，防并发污染)
 * ==================================================================== */
// 二进制巡线帧 (通道A) 的6步状态机变量
static uint8_t s_bin_rx_step = 0;
static uint8_t s_cmd_rx_step = 0;
static uint8_t s_tl = 0, s_th = 0; // 目标值低/高字节
static uint8_t s_cl = 0, s_ch = 0; // 当前值低/高字节

// 在 vision.c 顶部定义全局共享变量 (别忘了在 vofa.h 或 vision.h 加上 extern)
char g_Cmd_Trace_Buffer[128] = "Trace: ";
bool g_Flag_Send_Trace = false;
uint8_t s_trace_len = 7;
uint8_t s_last_traced_cmd = 0;

// 病房号 (通道C) 的状态机变量
static bool s_wait_for_room_num = false;

/* ====================================================================
 * 3. 核心解包引擎：Vision_Receive_Byte_ISR
 * ==================================================================== */
void Vision_Receive_Byte_ISR(uint8_t rx_data) {
    
    // -------------------------------------------------------------
    // 【通道 C】：目标病房通报帧 (N + 1~8)
    // -------------------------------------------------------------
    if (s_wait_for_room_num) {
        if (rx_data >= '1' && rx_data <= '8') {
            g_Target_Room = rx_data - '0';
        }
        s_wait_for_room_num = false;
        return;
    }
    if (rx_data == 'N') {
        s_wait_for_room_num = true;
        s_bin_rx_step = 0; // 熔断其他通道
        return;
    }
    // -------------------------------------------------------------
    // 通道 B: 离散动作指令帧 (A/D/S/W)
    // -------------------------------------------------------------
// -------------------------------------------------------------
    // 【通道 B】: 高优先级动作指令帧 (格式: C + 指令 + M)
    // -------------------------------------------------------------
    if (s_cmd_rx_step > 0) {
        // 处于指令匹配期，严密封锁数据，绝不漏给下方的坐标解析
        if (s_cmd_rx_step == 1) {
            // 验证是否为合法动作集
            if (rx_data == 'A' || rx_data == 'D' || rx_data == 'S' || 
                rx_data == 'W' || rx_data == 'G' || rx_data == 'P') {
                s_temp_cmd = rx_data; // 暂存在沙盒中，等待 M 确认
                s_cmd_rx_step = 2;
                return;
            } else {
                // 非法指令，匹配破裂
                s_cmd_rx_step = 0; 
                // 挽救机制：万一这个错乱的字符刚好是新一轮的包头 'C'，立刻重启匹配
                if (rx_data == 'C') { s_cmd_rx_step = 1; return; }
            }
        } 
        else if (s_cmd_rx_step == 2) {
            if (rx_data == 'M') {
                // 【终极匹配成功】：确认帧尾无误！
                g_Pi_Cmd = s_temp_cmd; 
                // 【VOFA 追踪器】：仅在有效新指令到达时记录，防止同指令高频刷爆
                if (g_Pi_Cmd != s_last_traced_cmd) {
                    if (s_trace_len < 120) {
                        if (s_trace_len > 6) { g_Cmd_Trace_Buffer[s_trace_len++] = ','; }
                        g_Cmd_Trace_Buffer[s_trace_len++] = g_Pi_Cmd;
                        //g_Cmd_Trace_Buffer[s_trace_len] = '\0';
                    }
                    s_last_traced_cmd = g_Pi_Cmd;
                    g_Flag_Send_Trace = true; // 通知主循环异步打印
                }
                // 瞬间熔断其他所有正在接收的状态机，动作指令优先级最高！
                s_bin_rx_step = 0;
                s_wait_for_room_num = false;
                s_cmd_rx_step = 0;
                return;
            } else {
                // 帧尾丢失 (比如发成了 C P X)，匹配破裂
                s_cmd_rx_step = 0;
                if (rx_data == 'C') { s_cmd_rx_step = 1; return; }
            }
        }
    } 
    else if (rx_data == 'C') {
        // 捕获到强力包头，正式挂起坐标解析，进入沙盒模式
        s_cmd_rx_step = 1;
        s_bin_rx_step = 0; // 熔断其他通道
        return;
    }
    // -------------------------------------------------------------
    // 【通道 A】：6字节小端二进制坐标帧 (0xAA + TL + TH + CL + CH + 0x55)
    // -------------------------------------------------------------
    switch (s_bin_rx_step) {
        case 0:
            if (rx_data == 0xAA) {
                s_bin_rx_step = 1;
            }
            break;
            
        case 1:
            s_tl = rx_data; // 接收 Target LSB
            s_bin_rx_step = 2;
            break;
            
        case 2:
            s_th = rx_data; // 接收 Target MSB
            s_bin_rx_step = 3;
            break;
            
        case 3:
            s_cl = rx_data; // 接收 Current LSB
            s_bin_rx_step = 4;
            break;
            
        case 4:
            s_ch = rx_data; // 接收 Current MSB
            s_bin_rx_step = 5;
            break;
            
        case 5:
            if (rx_data == 0x55) {
                // 1. 小端字节序拼接，还原出发送端的 16 位整型
                int16_t target_int  = (int16_t)((s_th << 8) | s_tl);
                int16_t current_int = (int16_t)((s_ch << 8) | s_cl);
                
                // 2. 根据 Python 端的协议，除以 10.0f 还原出真实浮点值
                target_f  = (float)target_int / 10.0f;
                current_f = (float)current_int / 10.0f;
                
                // 3. 【核心映射】算出物理误差值喂给外环 PID
                g_Vision_Offset = target_f - current_f;
                g_Vision_Updated = true;
            }
            // 无论包尾是否匹配，重置状态机以接收下一帧
            s_bin_rx_step = 0; 
            break;
            
        default:
            s_bin_rx_step = 0;
            break;
    }
}

