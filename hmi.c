#include "hmi.h"
#include "vision.h"
#include "bsp_mpu6050.h"
#include "pid.h"
#include <stdio.h> // 需要使用 sprintf 格式化字符串

static uint8_t  hmi_rx_buf[HMI_RX_MAX_LEN];
static volatile uint16_t hmi_rx_idx = 0;
static uint8_t  hmi_ff_count = 0; // 连续 0xFF 计数器
static volatile bool     hmi_frame_ready = false;
uint8_t g_HMI_Task_State = 0;

static void HMI_Run_Task_Command(char task_id) {
    /* [2026-06-12] HMI TASK1/2/3:
     * Reuse VOFA RST semantics, force FSM back to IDLE, then notify the
     * Raspberry Pi through UART_PI with T1/T2/T3.
     */
    FSM_Init();
    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
    PID_Clear(&pid_angle_Z); PID_Clear(&pid_vision);
    g_HMI_Task_State = (uint8_t)(task_id - '0');

    DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'T');
    DL_UART_Main_transmitDataBlocking(UART_PI_INST, task_id);
    DL_UART_Main_transmitDataBlocking(UART_PI_INST, '\r');  // ← 补上
    DL_UART_Main_transmitDataBlocking(UART_PI_INST, '\n');  // ← 补上
}

/* ====================================================================
 * 操作算子：串口屏单字节接收状态机 (非阻塞，无等待)
 * 物理映射：识别 0xFF 0xFF 0xFF 帧尾
 * ==================================================================== */
void HMI_Receive_Byte_ISR(uint8_t rx_data) {
    if (hmi_rx_idx < (HMI_RX_MAX_LEN - 1)) {
        hmi_rx_buf[hmi_rx_idx++] = rx_data;
        hmi_rx_buf[hmi_rx_idx] = '\0';

        /* [2026-06-12] HMI button events use existing screen style:
         *   prints "GO",2 / prints "TASK1",5
         * These send fixed-length ASCII payloads and do not necessarily append
         * the 0xFF 0xFF 0xFF terminator that MCU->HMI commands require.  Keep
         * the old terminator parser below, but also accept known button tokens
         * as soon as their complete payload has arrived.
         */
        if ((hmi_rx_idx == 2 && strncmp((char*)hmi_rx_buf, "GO", 2) == 0) ||
            (hmi_rx_idx == 5 && strncmp((char*)hmi_rx_buf, "TASK", 4) == 0 &&
             hmi_rx_buf[4] >= '1' && hmi_rx_buf[4] <= '3')) {
            hmi_frame_ready = true;
            hmi_ff_count = 0;
            return;
        }
        
        // 帧尾检测器
        if (rx_data == 0xFF) {
            hmi_ff_count++;
            if (hmi_ff_count == 3) {
                hmi_rx_buf[hmi_rx_idx - 3] = '\0'; // 剔除 0xFF，转为纯字符串
                hmi_frame_ready = true;
                hmi_ff_count = 0;
            }
        } else {
            hmi_ff_count = 0; // 一旦中断，重新计数
        }
    } else {
        hmi_rx_idx = 0; // 溢出重置
        hmi_ff_count = 0;
    }
}

/* ====================================================================
 * 操作算子：串口屏指令解码树
 * ==================================================================== */
void HMI_Parse_Command(void) {
    if (!hmi_frame_ready) return;
    if (strncmp((char*)hmi_rx_buf, "GO", 2) == 0) {
            // 通过修改 g_Pi_Cmd 伪造一个发车指令，复用 FSM 逻辑
            g_Pi_Cmd = 'G'; 
    } else if (strncmp((char*)hmi_rx_buf, "TASK1", 5) == 0) {
            HMI_Run_Task_Command('1');
    } else if (strncmp((char*)hmi_rx_buf, "TASK2", 5) == 0) {
            HMI_Run_Task_Command('2');
    } else if (strncmp((char*)hmi_rx_buf, "TASK3", 5) == 0) {
            HMI_Run_Task_Command('3');
    }

    hmi_rx_idx = 0;
    hmi_rx_buf[0] = '\0';
    hmi_frame_ready = false;
}

/* ====================================================================
 * 辅助发送算子：向 HMI 发送带 0xFF 帧尾的字符串
 * ==================================================================== */
void HMI_Send_Raw_String(const char *str) {
    while (*str) {
        DL_UART_Main_transmitDataBlocking(UART_HMI_INST, *str++);
    }
    DL_UART_Main_transmitDataBlocking(UART_HMI_INST, 0xFF);
    DL_UART_Main_transmitDataBlocking(UART_HMI_INST, 0xFF);
    DL_UART_Main_transmitDataBlocking(UART_HMI_INST, 0xFF);
}

/* ====================================================================
 * 界面刷新算子：同步系统状态至屏幕
 * 建议 500ms 左右调用一次，避免阻塞主循环
 * ==================================================================== */
void HMI_Update_UI_State(SystemState_t state) {
    switch (state) {
        case STATE_IDLE:          HMI_Send_Raw_String("t_stat.txt=\"IDLE\""); break;
        case STATE_FORWARD_TRACK: HMI_Send_Raw_String("t_stat.txt=\"FWD_TRACK\""); break;
        case STATE_REVERSE_TRACK: HMI_Send_Raw_String("t_stat.txt=\"REV_TRACK\""); break;
        case STATE_TRANSITION_FWD:HMI_Send_Raw_String("t_stat.txt=\"TR_FWD\""); break;
        case STATE_TRANSITION_REV: HMI_Send_Raw_String("t_stat.txt=\"TR_REV\""); break;
        case STATE_TURN_ABS:      HMI_Send_Raw_String("t_stat.txt=\"TURN_ABS\""); break;
        case STATE_BRAKE_PHARMACY:HMI_Send_Raw_String("t_stat.txt=\"BRAKE\""); break;
        case STATE_WAIT_INTERSECTION: HMI_Send_Raw_String("t_stat.txt=\"WAIT\""); break;
        case STATE_PRE_STOP_PHARMACY: HMI_Send_Raw_String("t_stat.txt=\"PRE_STOP\""); break;
        /* ==================================================================== */
        default: break;
    }
}

/* ====================================================================
 * 3. [业务层] 刷新目标病房号 (1~8)
 * 协议格式: t_room.txt="目标: X"
 * ==================================================================== */
void HMI_Update_Target_Room(uint8_t room_num) {
    char hmi_cmd[32];
    snprintf(hmi_cmd, sizeof(hmi_cmd), "t_room.txt=\"Room:%d\"", room_num);
    HMI_Send_Raw_String(hmi_cmd);
}

/* ====================================================================
 * 4. [业务层] 刷新传感器 Debug 数据 (视觉 & 陀螺仪)
 * 协议格式: t_pi.txt="xxx" \ t_gyro.txt="xxx.xx"
 * ==================================================================== */
void HMI_Update_Debug_Data(float vision_x, float gyro_yaw) {
    char hmi_cmd[32];
    
    // 刷新视觉 X 坐标到名为 t_pi 的 Text 控件
    snprintf(hmi_cmd, sizeof(hmi_cmd), "t_pi.txt=\"%d\"", (int)vision_x);
    HMI_Send_Raw_String(hmi_cmd);
    
    // 刷新陀螺仪角度到名为 t_gyro 的 Text 控件 (保留1位小数)
    snprintf(hmi_cmd, sizeof(hmi_cmd), "t_gyro.txt=\"%.1f\"", gyro_yaw);
    HMI_Send_Raw_String(hmi_cmd);
}
