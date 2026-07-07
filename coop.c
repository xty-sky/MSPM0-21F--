#include "coop.h"
#include "vision.h" // 依赖引入：为了在收到指令后，能够写入 g_Target_Room
#include <stdio.h>  // 依赖引入：为了使用 sprintf

/* ====================================================================
 * 全局并发标志位物理内存分配 (定义)
 * ==================================================================== */
volatile bool g_Coop_Remote_Go_Flag = false;

/* ====================================================================
 * 解析状态机内部私有变量 (对外部绝对隐藏)
 * ==================================================================== */
static bool s_coop_is_receiving = false;
static uint8_t s_coop_rx_step = 0;

/* ====================================================================
 * 发送算子：格式 "@GO_x$" (其中 x 为 1~8 的病房号)
 * ==================================================================== */
void Coop_Send_Start_Cmd(uint8_t room_num) {
    char send_buf[10];
    sprintf(send_buf, "@GO_%d$", room_num);
    
    // 冗余盲发 3 次，对抗 2.4G 空间干扰
    for (int i = 0; i < 3; i++) {
        for(int j = 0; send_buf[j] != '\0'; j++) {
            DL_UART_Main_transmitDataBlocking(UART_COOP_INST, send_buf[j]);
        }
    }
}

/* ====================================================================
 * 接收算子：纯状态机，无数组无阻塞
 * ==================================================================== */
void Coop_Receive_Byte_ISR(uint8_t rx_data) {
    // 起始符拦截
    if (rx_data == '@') {
        s_coop_is_receiving = true;
        s_coop_rx_step = 0;
        return;
    }
    
    // 序列推进与校验
    if (s_coop_is_receiving) {
        if (s_coop_rx_step == 0 && rx_data == 'G') { s_coop_rx_step++; }
        else if (s_coop_rx_step == 1 && rx_data == 'O') { s_coop_rx_step++; }
        else if (s_coop_rx_step == 2 && rx_data == '_') { s_coop_rx_step++; }
        else if (s_coop_rx_step == 3) {
            // 捕获数字 1~8，直接覆盖写入全局病房号
            if (rx_data >= '1' && rx_data <= '8') {
                g_Target_Room = rx_data - '0';
                s_coop_rx_step++;
            } else {
                s_coop_is_receiving = false; 
            }
        }
        else if (s_coop_rx_step == 4 && rx_data == '$') {
            // 完美匹配一帧，扣动发车扳机！
            g_Coop_Remote_Go_Flag = true;
            s_coop_is_receiving = false;
        }
        else {
            s_coop_is_receiving = false; // 序列错误，中断
        }
    }
}