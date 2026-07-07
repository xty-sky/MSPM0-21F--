#include "fsm.h"
#include "mydrive21E.h"
#include "bsp_mpu6050.h"
#include "pid.h"
#include "mymain.h"
#include <math.h>

/*
 * fsm_debug.c — 陀螺仪 PD 参数调试专用状态机 (_Debug 后缀, 与 fsm.c 无冲突)
 * 简化工况: STATE_IDLE / STATE_FORWARD_TRACK / STATE_TURN_ABS
 * 核心原则: yaw 经 mymain4.c 解卷绕后为连续广义角度,
 *           仅对误差做 Normalize_Angle 取最短路径,
 *           actual_rate 由 g_Gyro_Z_Rate 提供真实角速度供 D 项阻尼。
 */

SystemState_t s_current_state_debug = STATE_IDLE;

const char *SystemStateStr_Debug[] = {
    "STATE_IDLE",
    "STATE_FORWARD_TRACK",
    "STATE_WAIT_INTERSECTION",
    "STATE_TRANSITION_FWD",
    "STATE_TRANSITION_REV",
    "STATE_TURN_ABS",
    "STATE_PRE_STOP_PHARMACY",
    "STATE_BRAKE_PHARMACY",
    "STATE_REVERSE_TRACK",
    "STATE_ANGLE_HOLD",
    "STATE_TURN_HOLD"
};

float g_FSM_Base_Speed_Debug = 0.5f;
float g_FSM_Reverse_Speed_Debug = -0.5f;  // 倒车速度 (负值)
float g_FSM_Turn_Tolerance_Debug = 2.0f;
float g_Track_Direction = 1.0f;             // +1=正向, -1=反向

static float s_target_yaw_abs = 0.0f;
static float s_pending_turn_angle = 0.0f;
static float s_yaw_error_debug = 0.0f;    // 最近一帧 yaw 误差 (供遥测)

float out_L_debug = 0, out_R_debug = 0;

volatile bool g_Flag_Gyro_Tune_Mode = false;
static float s_gyro_tune_target = 0.0f;

/* ====================================================================
 * 初始化
 * ==================================================================== */
void FSM_Init_Debug(void) {
    s_current_state_debug = STATE_IDLE;
    s_target_yaw_abs = 0.0f;
    s_pending_turn_angle = 0.0f;
    /* 【RST修复】: 偏置捕获移至 mymain4.c s_yaw_first 分支, 与首次DMP读取同步 */
    g_Yaw_Offset = 0.0f;
    yaw = 0.0f;
    g_Weight_Vision_Fwd = 0.0f;  // 默认纯陀螺, 视觉需手动用 WV= 开启
    g_Weight_Gyro_Fwd   = 1.0f;
    g_Weight_Vision_Rev = 0.0f;  // 反向同理
    g_Weight_Gyro_Rev   = 1.0f;
    g_Track_Direction   = 1.0f;
    Yaw_Integrator_Reset_Debug();
    Motor_SetPWM(0, 0);
    Motor_SetPWM(1, 0);
    delay_cycles(2000000);
}

void FSM_Init_first_Debug(void) {
    FSM_Init_Debug();
}

/* ====================================================================
 * 查询接口
 * ==================================================================== */
SystemState_t FSM_Get_State_Debug(void) {
    return s_current_state_debug;
}

float FSM_Get_Target_Yaw_Debug(void) {
    if (g_Flag_Gyro_Tune_Mode) return s_gyro_tune_target;
    return s_target_yaw_abs;
}

float FSM_Get_Yaw_Error_Debug(void) {
    return s_yaw_error_debug;
}

/* ====================================================================
 * 转向触发: 从当前广义航向累加 delta_angle
 * ==================================================================== */
void FSM_Trigger_Turn_Debug(SystemState_t trans_type, float delta_angle) {
    s_current_state_debug = trans_type;
    float current_yaw_relative = (yaw - g_Yaw_Offset);
    s_pending_turn_angle = delta_angle;
    s_target_yaw_abs = current_yaw_relative + delta_angle;
    PID_Clear(&pid_angle_Z);
}

void FSM_Trigger_Reverse_Start_Debug(void) {
    s_current_state_debug = STATE_REVERSE_TRACK;
}

/* ====================================================================
 * 主状态机 (每 10ms 调用)
 * ==================================================================== */
void FSM_Update_10ms_Debug(void) {

    // 当前连续广义角度
    float current_yaw = (yaw - g_Yaw_Offset);

    // 陀螺仪性能测试模式 (优先级最高)
    if (g_Flag_Gyro_Tune_Mode) {
        /* Normalize_Angle 定最短路径方向; 连续 error 供遥测 */
        float yaw_error_norm = Normalize_Angle(s_gyro_tune_target - current_yaw);
        float aligned_target  = current_yaw + yaw_error_norm;
        s_yaw_error_debug = yaw_error_norm;

        float gyro_comp = PID_Calc_PD_Outer_With_Rate(
            &pid_angle_Z, aligned_target, current_yaw, g_Gyro_Z_Rate);

        Kinematics_Update_Target(0.0f, gyro_comp);
        out_L_debug = PID_Calc_PI_Inner(&pid_speed_L, g_Target_Pulse_L, (float)g_Chassis.speed_L);
        out_R_debug = PID_Calc_PI_Inner(&pid_speed_R, g_Target_Pulse_R, (float)g_Chassis.speed_R);
        Motor_SetPWM(0, (int16_t)out_L_debug);
        Motor_SetPWM(1, (int16_t)out_R_debug);
        return;
    }

    switch (s_current_state_debug) {

    case STATE_IDLE:
        s_yaw_error_debug = Normalize_Angle(0.0f - current_yaw);
        Motor_SetPWM(0, 0);
        Motor_SetPWM(1, 0);
        PID_Clear(&pid_speed_L);
        PID_Clear(&pid_speed_R);
        PID_Clear(&pid_angle_Z);
        g_Target_Pulse_L = 0.0f;
        g_Target_Pulse_R = 0.0f;
        return;

    case STATE_FORWARD_TRACK:
    {
        // 方向感知: 正向/反向用不同基速、极性、权重
        float polarity  = (g_Track_Direction > 0) ? 1.0f : -1.0f;
        float base_spd  = (g_Track_Direction > 0) ? g_FSM_Base_Speed_Debug : g_FSM_Reverse_Speed_Debug;
        float weight_v  = (g_Track_Direction > 0) ? g_Weight_Vision_Fwd : g_Weight_Vision_Rev;
        float weight_g  = (g_Track_Direction > 0) ? g_Weight_Gyro_Fwd   : g_Weight_Gyro_Rev;

        // 陀螺仪航向误差 (正反向共用)
        float yaw_error_raw = s_target_yaw_abs - current_yaw;
        float yaw_error_min  = Normalize_Angle(yaw_error_raw);
        float aligned_target = current_yaw + yaw_error_min;
        s_yaw_error_debug = yaw_error_min;

        // 视觉数据刷新检测
        static float vision_err = 0.0f;
        bool vision_ok = false;
        if (g_Vision_Updated) {
            vision_err = g_Vision_Offset;
            g_Vision_Updated = false;
            vision_ok = true;
        }

        float outer_comp;
        if ( weight_v > 0.001f && weight_g >  0.001f ) {//vision_ok &&
            outer_comp = Fusion_Outer_Loop(
                vision_err, aligned_target, current_yaw, g_Gyro_Z_Rate,
                polarity, weight_v, weight_g);
        } else if(weight_g >  0.001f){
            outer_comp = PID_Calc_PD_Outer_With_Rate(
                &pid_angle_Z, aligned_target, current_yaw, g_Gyro_Z_Rate);
        } else if(weight_v >  0.001f){
            outer_comp = PID_Calc_PD_Outer_Discrete(&pid_vision, 0.0f, g_Vision_Offset);
        } else outer_comp = 0.0f;

        Kinematics_Update_Target(base_spd, outer_comp);
        out_L_debug = PID_Calc_PI_Inner(&pid_speed_L, g_Target_Pulse_L, (float)g_Chassis.speed_L);
        out_R_debug = PID_Calc_PI_Inner(&pid_speed_R, g_Target_Pulse_R, (float)g_Chassis.speed_R);
        Motor_SetPWM(0, (int16_t)out_L_debug);
        Motor_SetPWM(1, (int16_t)out_R_debug);
    }
    break;

    case STATE_TURN_ABS:
    {
        /* [2026-06-01] PD方向: Normalize_Angle 保证最短路径
         * 完成判定: 连续 yaw 差补偿 DMP 高速少报 */
        float yaw_error_norm = Normalize_Angle(s_target_yaw_abs - current_yaw);
        float aligned_target  = current_yaw + yaw_error_norm;
        s_yaw_error_debug = yaw_error_norm;

        float gyro_comp = PID_Calc_PD_Outer_With_Rate(
            &pid_angle_Z, aligned_target, current_yaw, g_Gyro_Z_Rate);

        Kinematics_Update_Target(0.0f, gyro_comp);
        out_L_debug = PID_Calc_PI_Inner(&pid_speed_L, g_Target_Pulse_L, (float)g_Chassis.speed_L);
        out_R_debug = PID_Calc_PI_Inner(&pid_speed_R, g_Target_Pulse_R, (float)g_Chassis.speed_R);
        Motor_SetPWM(0, (int16_t)out_L_debug);
        Motor_SetPWM(1, (int16_t)out_R_debug);

        /* 完成: 连续差值 = 真实累积转角, 补偿 DMP 高速少报 */
        if (fabs(s_target_yaw_abs - current_yaw) <= g_FSM_Turn_Tolerance_Debug) {
            s_current_state_debug = STATE_FORWARD_TRACK;
        }
    }
    break;

    case STATE_ANGLE_HOLD:
    {
        float yaw_error_norm = Normalize_Angle(s_target_yaw_abs - current_yaw);
        float aligned_target  = current_yaw + yaw_error_norm;
        s_yaw_error_debug = yaw_error_norm;

        float gyro_comp = PID_Calc_PD_Outer_With_Rate(
            &pid_angle_Z, aligned_target, current_yaw, g_Gyro_Z_Rate);

        Kinematics_Update_Target(0.0f, gyro_comp);
        out_L_debug = PID_Calc_PI_Inner(&pid_speed_L, g_Target_Pulse_L, (float)g_Chassis.speed_L);
        out_R_debug = PID_Calc_PI_Inner(&pid_speed_R, g_Target_Pulse_R, (float)g_Chassis.speed_R);
        Motor_SetPWM(0, (int16_t)out_L_debug);
        Motor_SetPWM(1, (int16_t)out_R_debug);
    }
    break;

    case STATE_TURN_HOLD:
    {
        float yaw_error_norm = Normalize_Angle(s_target_yaw_abs - current_yaw);
        float aligned_target  = current_yaw + yaw_error_norm;
        s_yaw_error_debug = yaw_error_norm;

        float gyro_comp = PID_Calc_PD_Outer_With_Rate(
            &pid_angle_Z, aligned_target, current_yaw, g_Gyro_Z_Rate);

        Kinematics_Update_Target(0.0f, gyro_comp);
        out_L_debug = PID_Calc_PI_Inner(&pid_speed_L, g_Target_Pulse_L, (float)g_Chassis.speed_L);
        out_R_debug = PID_Calc_PI_Inner(&pid_speed_R, g_Target_Pulse_R, (float)g_Chassis.speed_R);
        Motor_SetPWM(0, (int16_t)out_L_debug);
        Motor_SetPWM(1, (int16_t)out_R_debug);
        // 不检查误差, 不切出, 永久保持
    }
    break;

    default:
        s_current_state_debug = STATE_IDLE;
        Motor_SetPWM(0, 0);
        Motor_SetPWM(1, 0);
        break;
    }
}

/* ====================================================================
 * 键盘调试指令
 * ==================================================================== */
void FSM_Debug_Command(char cmd) {
    switch (cmd) {
    case 'G':
        g_Flag_Gyro_Tune_Mode = false;
        s_current_state_debug = STATE_FORWARD_TRACK;
        s_target_yaw_abs = 0.0f;
        PID_Clear(&pid_angle_Z);
        break;
    case 'D':
        g_Flag_Gyro_Tune_Mode = false;
        FSM_Trigger_Turn_Debug(STATE_TURN_ABS, -90.0f);
        break;
    case 'A':
        g_Flag_Gyro_Tune_Mode = false;
        FSM_Trigger_Turn_Debug(STATE_TURN_ABS, 90.0f);
        break;
    /* [2026-05-31 新增] 180°原地掉头: 测试陀螺仪PD在大角度转弯时的响应
     * 与主构建 fsm.c 中药房180°转弯逻辑对应, 便于在线整定 pid_angle_Z 参数 */
    case 'B':
        g_Flag_Gyro_Tune_Mode = false;
        FSM_Trigger_Turn_Debug(STATE_TURN_ABS, 180.0f);
        break;
    case 'S':
        g_Flag_Gyro_Tune_Mode = false;
        s_current_state_debug = STATE_IDLE;
        break;
    default:
        break;
    }
}

/* ====================================================================
 * 陀螺仪性能测试
 * ==================================================================== */
void FSM_Enter_Gyro_Tune_Mode(float target_angle) {
    g_Flag_Gyro_Tune_Mode = true;
    float current_yaw_relative = (yaw - g_Yaw_Offset);
    s_gyro_tune_target = current_yaw_relative + target_angle;
    s_target_yaw_abs = s_gyro_tune_target;
    s_current_state_debug = STATE_IDLE;
    PID_Clear(&pid_angle_Z);
    PID_Clear(&pid_speed_L);
    PID_Clear(&pid_speed_R);
}

void FSM_Exit_Gyro_Tune_Mode(void) {
    g_Flag_Gyro_Tune_Mode = false;
    s_current_state_debug = STATE_IDLE;
    Motor_SetPWM(0, 0);
    Motor_SetPWM(1, 0);
    PID_Clear(&pid_angle_Z);
}

void FSM_Set_Gyro_Target(float target_angle) {
    s_gyro_tune_target = target_angle;
}

/* ====================================================================
 * 角度锁定校准: 静止, 锁定当前航向, 抗扰动回正
 * ==================================================================== */
void FSM_Enter_Angle_Hold(void) {
    g_Flag_Gyro_Tune_Mode = false;
    float current_yaw_relative = (yaw - g_Yaw_Offset);
    s_target_yaw_abs = current_yaw_relative;  // 锁定当前位置为目标
    s_current_state_debug = STATE_ANGLE_HOLD;
    PID_Clear(&pid_angle_Z);
    PID_Clear(&pid_speed_L);
    PID_Clear(&pid_speed_R);
}

/* ====================================================================
 * 转角锁定: 转向完成后保持目标角, 不制动, 不切出
 * ==================================================================== */
void FSM_Trigger_Turn_Hold_Debug(SystemState_t trans_type, float delta_angle) {
    g_Flag_Gyro_Tune_Mode = false;
    s_current_state_debug = trans_type;
    float current_yaw_relative = (yaw - g_Yaw_Offset);
    s_target_yaw_abs = current_yaw_relative + delta_angle;
    PID_Clear(&pid_angle_Z);
}
