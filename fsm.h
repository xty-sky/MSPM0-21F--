#ifndef FSM_H
#define FSM_H

#include "ti_msp_dl_config.h"
#include "pid.h"
extern bool action_put_medicine ;
extern float out_L;
extern float out_R;
extern float g_Turn_Entry_Drift;
typedef enum {
    STATE_IDLE = 0,         
    STATE_FORWARD_TRACK,
    STATE_WAIT_INTERSECTION,   // 【新增】: 十字路口急刹挂起态
    STATE_TRANSITION_FWD,
    STATE_TRANSITION_REV,
    STATE_TURN_ABS,         
    STATE_PRE_STOP_PHARMACY,   // 【新增】: 药房预停延时态
    STATE_BRAKE_PHARMACY,
    STATE_REVERSE_TRACK,
    STATE_ANGLE_HOLD,          // 角度锁定校准态 (基速=0, 抗扰动)
    STATE_TURN_HOLD            // 转角锁定态 (转到目标后保持, 不切出)
} SystemState_t;

extern char hst[32];
extern uint8_t stt;
extern uint8_t g_Pi_Cmd;
extern uint8_t last_g_Pi_Cmd;
extern SystemState_t s_current_state;
// 在 fsm.h 中声明：告诉所有包含此头文件的 .c 文件，"在别处有一个名叫 SystemStateStr 的数组"
extern const char* SystemStateStr[];

// 【全量 Extern 声明：状态机全局参数】
extern float g_FSM_Base_Speed;          
extern float g_FSM_Reverse_Speed;       
extern uint16_t g_FSM_Trans_Fwd_Delay;  
extern uint16_t g_FSM_Trans_Rev_Delay;  
extern float g_FSM_Trans_Rev_Speed;
extern float g_FSM_Turn_Tolerance;
extern uint16_t g_FSM_Right_Brake_Delay; /* [2026-05-28] 右轮延时刹车周期数 */
extern uint16_t g_FSM_Brake_Cycles;       /* [2026-05-28] 非阻塞刹车拍数 ×10ms */
extern uint16_t g_FSM_Cooldown_Cycles;    /* [2026-05-28] 路口冷却拍数 ×10ms */
extern float g_Vision_Calib_Threshold;    /* [2026-05-28] 视觉校正触发阈值(像素) */      

extern uint16_t g_FSM_Pre_Stop_Delay;   // 【新增】: 预停止延时参数 (VOFA 可调)

extern uint8_t cmd;
// 【全量 Extern 声明：操作算子】
extern void FSM_Init_first(void);
extern void FSM_Init(void);
extern void FSM_Update_10ms(void);
extern SystemState_t FSM_Get_State(void);
extern float FSM_Get_Yaw_Error(void);
extern float FSM_Get_Target_Yaw(void);
extern bool FSM_Is_Turn_Critical_Window(void);
extern void FSM_Trigger_Turn(SystemState_t trans_type, float delta_angle);
extern void FSM_Trigger_Pharmacy(void);
extern void FSM_Trigger_Reverse_Start(void);
extern void FSM_Apply_Reverse_Gyro_PID(void);   /* 反向循迹独立陀螺仪PD参数装载 */
extern void FSM_Vision_Calibrate_Heading(void); /* [2026-05-28] 视觉直行确认 → 校正陀螺航向 */

// 【新增】：陀螺仪性能测试接口 (定义在 fsm_debug.c)
extern volatile bool g_Flag_Gyro_Tune_Mode;
extern void FSM_Enter_Gyro_Tune_Mode(float target_angle);
extern void FSM_Exit_Gyro_Tune_Mode(void);
extern void FSM_Set_Gyro_Target(float target_angle);
extern void FSM_Debug_Command(char cmd);

// ====================================================================
// 【Debug 构建】: fsm_debug.c 专属声明 (_Debug 后缀, 与 fsm.c 无冲突)
// ====================================================================
extern SystemState_t s_current_state_debug;
extern const char* SystemStateStr_Debug[];
extern float g_FSM_Base_Speed_Debug;
extern float g_FSM_Turn_Tolerance_Debug;
extern float out_L_debug;
extern float out_R_debug;
extern float g_Track_Direction;          // +1=正向, -1=反向
extern float g_FSM_Reverse_Speed_Debug;  // 倒车基准速度

extern void FSM_Init_Debug(void);
extern void FSM_Init_first_Debug(void);
extern void FSM_Update_10ms_Debug(void);
extern SystemState_t FSM_Get_State_Debug(void);
extern float FSM_Get_Yaw_Error_Debug(void);
extern float FSM_Get_Target_Yaw_Debug(void);
extern void FSM_Trigger_Turn_Debug(SystemState_t trans_type, float delta_angle);
extern void FSM_Trigger_Reverse_Start_Debug(void);
extern void FSM_Enter_Angle_Hold(void);
extern void FSM_Trigger_Turn_Hold_Debug(SystemState_t trans_type, float delta_angle);

#endif
