#ifndef HMI_H
#define HMI_H

#include "ti_msp_dl_config.h"
#include "fsm.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define HMI_RX_MAX_LEN 32

// 全局暴露：当前选择的目标病房号 (1, 2, 3)
extern uint8_t g_Target_Room;
extern uint8_t g_HMI_Task_State;

// 核心操作算子接口
void HMI_Receive_Byte_ISR(uint8_t rx_data);
void HMI_Parse_Command(void);
void HMI_Send_Raw_String(const char *str);

extern void HMI_Update_Target_Room(uint8_t room_num);
extern void HMI_Update_Debug_Data(float vision_x, float gyro_yaw);
extern void HMI_Update_UI_State(SystemState_t state);

#endif // HMI_H
