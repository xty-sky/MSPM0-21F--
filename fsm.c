#include "fsm.h"
#include "hmi.h"
#include "bsp_mpu6050.h"
#include "vision.h"     // 【修复】：引入 cmd和 g_Vision_Offset
#include "mydrive21E.h" // 【修复】：引入 Motor_SetPWM 和 g_Chassis
#include "mymain.h"
#include "coop.h"
#include "vofa.h"

SystemState_t s_current_state = STATE_IDLE;
static bool s_Reset_Debounce_Flag = false;
// 【修订】：双计时器解耦
static uint16_t      s_trans_fwd_timer = 0; 
static uint16_t      s_trans_rev_timer = 0;
static uint16_t      s_trans_prestop_timer = 0; 

uint8_t cmd = 0;

const char *SystemStateStr[] = {
    "STATE_IDLE",
    "STATE_FORWARD_TRACK",
    "STATE_WAIT_INTERSECTION",
    "STATE_TRANSITION_FWD",
    "STATE_TRANSITION_REV",
    "STATE_TURN_ABS",
    "STATE_PRE_STOP_PHARMACY",
    "STATE_BRAKE_PHARMACY",
    "STATE_REVERSE_TRACK"
};

char hst[32]={0};
uint8_t stt=0;
uint8_t last_g_Pi_Cmd=0;

bool action_put_medicine  = 0;
float out_L = 0,out_R = 0;
float g_Turn_Entry_Drift = 0.0f;
static float         s_target_yaw_abs = 0.0f;
static float         s_pending_turn_angle = 0.0f;
/* [2026-05-28] 上一次转弯完成时的干净目标航向 (90°整数倍), 作为下次转向的绝对基准 */
static float         s_last_cardinal_target = 0.0f;
// 全局调参变量初始化
float g_FSM_Base_Speed = 0.5f;
float g_FSM_Reverse_Speed = -0.5f;
uint16_t g_FSM_Trans_Fwd_Delay = 150;    // 正向过渡约 600ms (60×10ms)
uint16_t g_FSM_Trans_Rev_Delay = 150;    // 倒车过渡约 600ms (60×10ms)
float g_FSM_Trans_Rev_Speed = 0.06f;    /* [2026-06-01] 回滚: 旧值0.3→0.06, 过渡低速 */
float g_FSM_Turn_Tolerance = 2.0f;      /* [2026-06-11] 5.0→2.0, 回退严格死区 */
uint16_t g_FSM_Right_Brake_Delay = 0;
uint16_t g_FSM_Brake_Cycles    = 10;    /* [2026-06-01 修复] 旧值0→10, 刹车100ms */
uint16_t g_FSM_Cooldown_Cycles = 5;     /* [2026-06-01 修复] 旧值0→5, 冷却50ms */
/* 命令可接收延迟 = max(刹车, 右延) + 冷却 = max(10,0) + 5 = 15拍 = 150ms */
float g_Vision_Calib_Threshold = 5.0f;
uint16_t g_FSM_Pre_Stop_Delay = 20;
static float s_yaw_error = 0.0f;         // 最近一帧 yaw 误差 (供遥测)

/* 【漂移补偿】: 陀螺仪零漂检测与校准状态机 */
static float  s_idle_entry_yaw    = 0.0f;   /* 进入 IDLE 时的参考偏角 */
static bool   s_idle_first_entry  = true;   /* IDLE 首次进入标志 */
static float  s_brake_entry_yaw   = 0.0f;   /* 进入 BRAKE 时锁定的停车偏角 */
static bool   s_brake_first_entry = true;   /* BRAKE 首次进入标志 */
static float  s_brake_drift_bias  = 0.0f;   /* BRAKE期间DMP零漂累积, 发车前注入s_target_yaw_abs */
static float  s_wait_entry_yaw   = 0.0f;   /* [2026-05-28] 进入 WAIT_INTERSECTION 时刻的参考偏角 */
static bool   s_wait_first_entry = true;   /* [2026-05-28] WAIT_INTERSECTION 首次进入标志 */
static float  s_wait_drift_bias  = 0.0f;   /* [2026-05-28] WAIT期间DMP零漂累积, 转向触发时注入s_target_yaw_abs */
static uint16_t s_intersection_cooldown = 5; /* 路口刹车冷却: 500ms内禁止响应转向 */
static float    s_turn_start_yaw = 0.0f;      /* TURN_ABS入口时刻的连续yaw */
static uint8_t  s_turn_settle_cnt = 0;       /* 锁航倒计时(×10ms), 0=未到达/已结束 */
static uint8_t  s_post_turn_hold = 0;        /* [2026-06-11] FORWARD_TRACK陀螺仪过渡计时 */
static uint8_t  s_pharmacy_turns_left = 0;   /* [2026-06-10] 药房180°=2次D, 剩余次数 */
static float    s_pharmacy_ref_yaw = 0.0f;   /* [2026-06-11] 药房触发时刻的绝对基准 */
/* [2026-06-20] 药房停歪补偿: 视觉PD可能让停车瞬间车头偏离走廊方向。
 * 药房2x90掉头时用进入药房前最后一次视觉校准出的走廊航向作为180目标基准，
 * 不再用停车瞬间的实际yaw做rigid 90+90，否则会把停歪角原样带到回程。
 */
static bool     s_turn_target_override_valid = false;
static float    s_turn_target_override = 0.0f;
static bool     s_trans_entry_flag = false;  /* FSM_Trigger_Turn 已调用, 过渡态合法入口 */

/* ====================================================================
 * 非阻塞刹车计时器: 替代 delay_cycles() 硬阻塞, 防止 DMP FIFO 溢出
 * DMP FIFO 1024字节, 50%水位 ≈ 180ms 缓冲
 * delay_cycles(7200000)=225ms 必然触发溢出 → mpu_reset_fifo()
 *   → BIT_DMP_RST 复位 DMP 姿态解算器 → yaw 跳变 → 转弯超调
 * 改为 10ms 非阻塞计时: DMP 每周期持续读取, FIFO 永不溢出
 * ==================================================================== */
typedef enum {
    BRAKE_ACTION_NONE = 0,
    BRAKE_ACTION_TO_TURN_ABS,       /* 过渡结束 → 累加转角 → TURN_ABS */
    BRAKE_ACTION_SEND_CTM,          /* [2026-06-01] 药房: 2s制动→发CTM→续3s→循迹 */
    BRAKE_ACTION_TO_FORWARD_TRACK,  /* 转弯完成 → 记忆航向 → FORWARD_TRACK */
} BrakeAction_t;

static BrakeAction_t s_brake_action = BRAKE_ACTION_NONE;
static uint16_t      s_brake_timer  = 0;       /* 剩余 10ms 节拍数 */
/* [2026-05-28] 右轮延时刹车计时器: 停车时左轮先刹, 右轮延迟 s_right_delay 周期后再刹 */
static uint16_t      s_right_brake_delay_timer = 0;
/* [2026-05-28] 刹车期间转向命令缓存: 收到A/D不丢弃, 冷却结束后消费 */
static uint8_t       s_cached_turn_cmd = 0;
/* [2026-06-01] 药房180°转弯完成 → 通知树莓派 */
static bool          s_notify_pi_turn_done = false;

void FSM_Init_first(void) {
    s_current_state = STATE_IDLE;
    s_target_yaw_abs = 0.0f;
    s_pending_turn_angle = 0.0f;
    s_trans_fwd_timer = 0;
    s_trans_rev_timer = 0;
    s_brake_timer  = 0;
    s_brake_action = BRAKE_ACTION_NONE;
    s_brake_drift_bias = 0.0f;
    s_wait_drift_bias = 0.0f;
    s_last_cardinal_target = 0.0f;
    /* [2026-05-28] 全量复位新增静态变量, 防止 RST 后残留值污染状态机 */
    s_trans_entry_flag = false;
    s_cached_turn_cmd = 0;
    s_right_brake_delay_timer = 0;
    s_turn_target_override_valid = false;
    s_turn_target_override = 0.0f;
    yaw = 0.0f;
    Yaw_Integrator_Reset();
s_Reset_Debounce_Flag = true;
/* [2026-06-01] RST全量参数复位 — 所有VOFA可调变量回默认 */
g_FSM_Base_Speed           = 0.5f;
g_FSM_Reverse_Speed        = -0.5f;
g_FSM_Trans_Fwd_Delay      = 60;
g_FSM_Trans_Rev_Delay      = 60;
g_FSM_Trans_Rev_Speed      = 0.06f;
g_FSM_Turn_Tolerance       = 2.0f;
g_FSM_Right_Brake_Delay    = 0;
g_FSM_Brake_Cycles         = 10;
g_FSM_Cooldown_Cycles      = 5;
g_Vision_Calib_Threshold   = 5.0f;
g_FSM_Pre_Stop_Delay       = 20;
Motor_SetPWM(0, 0);Motor_SetPWM(1, 0);
delay_cycles(3200000);
}

void FSM_Init(void) {
    s_current_state = STATE_IDLE;
    s_target_yaw_abs = 0.0f;
    s_pending_turn_angle = 0.0f;
    s_trans_fwd_timer = 0;
    s_trans_rev_timer = 0;
    s_trans_prestop_timer = 0;
    s_brake_timer  = 0;
    s_brake_action = BRAKE_ACTION_NONE;
    s_brake_drift_bias = 0.0f;
    s_wait_drift_bias = 0.0f;
    s_last_cardinal_target = 0.0f;
    /* [2026-05-28] 全量复位新增静态变量 */
    s_trans_entry_flag = false;
    s_cached_turn_cmd = 0;
    s_right_brake_delay_timer = 0;
    s_turn_target_override_valid = false;
    s_turn_target_override = 0.0f;
    s_notify_pi_turn_done = false;
    yaw = 0.0f;
    Yaw_Integrator_Reset();
    s_Reset_Debounce_Flag = true;
/* [2026-06-01] RST全量参数复位 — 所有VOFA可调变量回默认 */
g_FSM_Base_Speed           = 0.5f;
g_FSM_Reverse_Speed        = -0.5f;
g_FSM_Trans_Fwd_Delay      = 60;
g_FSM_Trans_Rev_Delay      = 60;
g_FSM_Trans_Rev_Speed      = 0.06f;
g_FSM_Turn_Tolerance       = 2.0f;
g_FSM_Right_Brake_Delay    = 0;
g_FSM_Brake_Cycles         = 10;
g_FSM_Cooldown_Cycles      = 5;
g_Vision_Calib_Threshold   = 5.0f;
g_FSM_Pre_Stop_Delay       = 20;
Motor_SetPWM(0, 0);Motor_SetPWM(1, 0);
delay_cycles(3200000);
}
SystemState_t FSM_Get_State(void) {
    return s_current_state;
}

float FSM_Get_Yaw_Error(void) {
    return s_yaw_error;
}
float FSM_Get_Target_Yaw(void) {
    return s_target_yaw_abs;
}

bool FSM_Is_Turn_Critical_Window(void) {
    return (s_current_state == STATE_TURN_ABS) || (s_post_turn_hold > 0);
}

/* ====================================================================
 * 算子：反向循迹专用陀螺仪PD参数装载
 * 在进入 STATE_REVERSE_TRACK 时调用, 将独立参数写入 pid_angle_Z_rev
 * 与转弯90度模态的 pid_angle_Z 完全解耦, 互不干扰
 * ==================================================================== */
void FSM_Apply_Reverse_Gyro_PID(void) {
    /* 使用独立于转弯 PD 的反向循迹参数, out_max 复用 pid_angle_Z 的量程 */
    PID_Init(&pid_angle_Z_rev,
             g_Rev_Gyro_Kp,           /* P: 比例系数 */
             0.0f,                    /* I: 纯 PD, 无积分 */
             g_Rev_Gyro_Kd,           /* D: 微分系数 (依赖 g_Gyro_Z_Rate 真实角速度) */
             pid_angle_Z.out_max,     /* 输出限幅复用转弯 PD 配置 */
             0.0f,                    /* 算法设定无积分限幅 */
             pid_angle_Z.deadband,    /* 死区复用 */
             0.0f);                   /* 算法设定无积分带 */
    PID_Clear(&pid_angle_Z_rev);
}

/* ====================================================================
 * [2026-05-28] 算子：视觉直行确认 → 陀螺仪航向校正
 * FORWARD_TRACK 中，当视觉误差 |g_Vision_Offset| < 8 时，树莓派确认小车
 * 稳定在红线上。此时 GET_RELATIVE_YAW() 是真实直行航向，直接覆写
 * s_target_yaw_abs，消除行驶中 DMP 漂移累积。
 * 在 mymain.c 的 10ms 主循环中调用，位于 FSM_Update_10ms 之后。
 * ==================================================================== */
void FSM_Vision_Calibrate_Heading(void)
{
    /* [2026-06-11] 陀螺仪锁航过渡期(s_post_turn_hold>0)不更新target,
     * 否则Gyro_Only_Control的target被追着actual跑, 锁航空失效 */
    if (s_current_state == STATE_FORWARD_TRACK && s_post_turn_hold == 0
        && fabs(g_Vision_Offset) < g_Vision_Calib_Threshold) {
        s_target_yaw_abs = GET_RELATIVE_YAW();
    }
}

/* ====================================================================
 * 算子：触发转向并进入指定过渡态
 * ==================================================================== */
void FSM_Trigger_Turn(SystemState_t trans_type, float delta_angle)
{
    s_pending_turn_angle = delta_angle;
    s_notify_pi_turn_done = false;
    s_turn_start_yaw = GET_RELATIVE_YAW();
    s_turn_settle_cnt = 0;
    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
    PID_Clear(&pid_angle_Z); PID_Clear(&pid_angle_Z_rev);
    PID_Clear(&pid_vision);

    if (trans_type == STATE_TURN_ABS) {
        /* [2026-06-11] 直通TURN_ABS(药房2×D复用), 刹车500ms消前次D动量 */
        s_brake_timer  = 50;
        s_brake_action = BRAKE_ACTION_TO_TURN_ABS;
        return;
    }

    /* 十字路口: 过渡态直行→刹车→TURN_ABS */
    s_current_state = trans_type;
    s_target_yaw_abs = GET_RELATIVE_YAW();
    s_trans_fwd_timer = 0;
    s_trans_rev_timer = 0;
    s_trans_prestop_timer = 0;
    s_trans_entry_flag = true;
    if (trans_type == STATE_TRANSITION_FWD) {
        s_trans_fwd_timer = g_FSM_Trans_Fwd_Delay;
    } else {
        s_trans_rev_timer = g_FSM_Trans_Rev_Delay;
    }
}
/*
void FSM_Trigger_Pharmacy(void) {
    //抵达药房
    s_current_state = STATE_BRAKE_PHARMACY;
    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
}
*/

/* [2026-05-31 失能] FSM_Trigger_Reverse_Start: 药房 → 反向循迹 (已废弃)
 * 旧调用链: BRAKE_PHARMACY 药品边沿 → 本函数 → STATE_REVERSE_TRACK
 * 废弃原因: 倒车循迹稳定性差, 改为原地180°转弯 → STATE_FORWARD_TRACK
 * 漂移偏置注入算法 (s_brake_drift_bias → s_target_yaw_abs) 已被新流程复用
 * 保留代码供参考, 不再被任何路径调用 */
void FSM_Trigger_Reverse_Start(void) {
    s_current_state = STATE_REVERSE_TRACK;
    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
    /* 【漂移偏置注入】: BRAKE期间DMP零漂累积到 s_brake_drift_bias
     * 现在将其注入 s_target_yaw_abs, 抵消 GET_RELATIVE_YAW() 的漂移量
     * 保证 PID 误差 = target - current 不受 DMP 零漂影响
     * g_Yaw_Offset 全程未被修改, 绝对基准保持稳定 */
    s_target_yaw_abs += s_brake_drift_bias;
    s_brake_drift_bias = 0.0f;
    /* 【核心】: 进入反向循迹前, 将独立陀螺仪PD参数写入 pid_angle_Z_rev
     * 此后反向循迹全程使用 pid_angle_Z_rev, 与转弯90度的 pid_angle_Z 不绑定 */
    FSM_Apply_Reverse_Gyro_PID();
}

void FSM_Update_10ms(void) {
    // =========================================================
    // 🛡️ 1. 原子抓取层 (解决指令黏滞导致状态无限重启死锁)
    // =========================================================
    cmd = 0;
    __disable_irq();      // 关中断保原子性
    if (g_Pi_Cmd != 0) {
        cmd = g_Pi_Cmd;
        g_Pi_Cmd = 0;    // 抓取后立刻清空，绝不残留
    }
    __enable_irq();
if (cmd != 0 && cmd != last_g_Pi_Cmd) {
        if (stt < 32) {
            hst[stt++] = cmd;
        } else {
            stt = 0;
            hst[stt++] = cmd;
        }
        last_g_Pi_Cmd = cmd;
    }

        float current_vision_err = 0.0f;
    /* [2026-06-10 安全锁] 仅FORWARD_TRACK消费视觉数据,
     * 转弯/过渡/刹车等状态current_vision_err=0, 视觉完全隔离 */
    if (s_current_state == STATE_FORWARD_TRACK) {
        if (g_Vision_Updated) { current_vision_err = g_Vision_Offset; g_Vision_Updated = false; }
        else                 { current_vision_err = g_Vision_Offset; }
    }
    /* else: current_vision_err 保持 0.0f, 视觉不影响转弯 */

    /* ================================================================
     * [非阻塞刹车]: 替代 delay_cycles() 硬阻塞, 防止 DMP FIFO 溢出
     * 刹车期间每 10ms 递减计时器, 电机保持短路制动, 跳过全部 PID 计算
     * 计时器归零时执行延迟的状态迁移, 然后正常进入本轮 FSM 逻辑
     * ================================================================ */
    /* [2026-05-28] 右轮延时刹车: s_right_brake_delay_timer > 0 期间
     * 左轮立即制动, 右轮保持上一帧 PWM (惯性续流), 补偿左轮机械制动慢 */
    if (s_brake_timer > 0 || s_right_brake_delay_timer > 0) {
        /* [2026-05-28 修复] 并行倒数替代串行: 总阻塞=max(右延,主刹) 而非 右延+主刹 */
        // if (s_right_brake_delay_timer > 0) { s_right_brake_delay_timer--; }  /* 旧串行 */
        // else { s_brake_timer--; }
        if (s_right_brake_delay_timer > 0) s_right_brake_delay_timer--;
        if (s_brake_timer > 0) s_brake_timer--;
        /* 左轮始终立即制动 */
        Motor_SetPWM(0, 0);
        /* 右轮: 延时期间保持上一帧输出(不制动), 延时结束后制动 */
        if (s_right_brake_delay_timer == 0) {
            Motor_SetPWM(1, 0);
        }
        /* 两个计时器都归零才执行刹车到期动作 */
        if (s_brake_timer == 0 && s_right_brake_delay_timer == 0) {
            /* 刹车到期: 执行挂起的迁移动作 */
            if (s_brake_action == BRAKE_ACTION_TO_TURN_ABS) {
                /* [2026-06-10 重构] target = 当前偏角 + 期望转角
                 * 与调试层 fsm_debug.c 完全同构: current+delta
                 * 漂移自动抵消: current已包含静置期DMP漂移, target同步偏移
                 * → 物理转角=delta, 不受DMP零漂影响
                 * 冻结基准(s_wait_entry/brake_entry)仅遥测用, 不参与目标计算 */
                float turn_entry_yaw = GET_RELATIVE_YAW();
                g_Turn_Entry_Drift = turn_entry_yaw - s_turn_start_yaw;
                if (s_turn_target_override_valid) {
                    s_target_yaw_abs = s_turn_target_override;
                    s_turn_target_override_valid = false;
                } else {
                    s_target_yaw_abs = turn_entry_yaw + s_pending_turn_angle;
                }
                s_wait_drift_bias  = 0.0f;  /* [2026-06-10] 仅遥测, 转弯后清零 */
                s_brake_drift_bias = 0.0f;
                s_current_state = STATE_TURN_ABS;
                s_brake_action = BRAKE_ACTION_NONE;
                /* [2026-06-12] This is the real A/D turn entry after
                 * transition+brake.  Re-clear the gyro PD here, matching
                 * fsm_debug.c direct TURN_ABS entry instead of carrying the
                 * transition gyro-lock output into the first turn frame.
                 */
                PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                PID_Clear(&pid_angle_Z); PID_Clear(&pid_vision);
            } else if (s_brake_action == BRAKE_ACTION_SEND_CTM) {
                /* [2026-06-01] 药房180°: 2s→发CTM→续3s→循迹 */
                s_notify_pi_turn_done = false;
                DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'C');
                DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'T');
                DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'M');
                    DL_UART_Main_transmitDataBlocking(UART_PI_INST, '\r');  // ← 补上
                DL_UART_Main_transmitDataBlocking(UART_PI_INST, '\n');  // ← 补上

                s_brake_timer  = 30;  /* [2026-06-10] 200→30, 2s→300ms */
                s_brake_action = BRAKE_ACTION_TO_FORWARD_TRACK;
                /* 不执行 s_brake_action = NONE, 保留 FORWARD_TRACK */
            } else if (s_brake_action == BRAKE_ACTION_TO_FORWARD_TRACK) {
                /* [2026-06-10] 刹车到期复检: 偏差>2°→重回TURN_ABS修正 */
                float err = Normalize_Angle(s_target_yaw_abs - GET_RELATIVE_YAW());
                if (fabs(err) > g_FSM_Turn_Tolerance) {
                    /* 刹车期间偏离超限, 重入TURN_ABS让PD掰回,
                     * 不清速度PID→保留积分项, 小误差时也能输出足够力矩 */
                    s_current_state = STATE_TURN_ABS;
                    s_brake_action = BRAKE_ACTION_NONE;
                    s_turn_settle_cnt = 0;
                } else {
                    /* 偏差在容差内, 正常进入FORWARD_TRACK */
                    s_target_yaw_abs = GET_RELATIVE_YAW();
                    s_post_turn_hold = 50;  /* 500ms陀螺仪锁航过渡 */
                    s_current_state = STATE_FORWARD_TRACK;
                    s_brake_action = BRAKE_ACTION_NONE;
                    PID_Clear(&pid_speed_L); PID_Clear(&pid_speed_R);
                    PID_Clear(&pid_angle_Z); PID_Clear(&pid_vision);
                }
            }
            /* 刹车刚到期, 不 return, 继续执行本轮 FSM (新状态) */
        } else {
            /* 刹车未到期: 跳过本轮 FSM, 电机保持制动(左)/续流(右) */
            /* [2026-05-28] 刹车期间不丢弃 A/D/W, 缓存到冷却结束后消费 */
            if (cmd == 'A' || cmd == 'D' || cmd == 'W') {
                s_cached_turn_cmd = cmd;
            }
            return;
        }
    }

/* ====================================================================
     * 【核心注入】：光电对管 移位寄存器防抖与边沿检测 (Shift Register Debounce)
     * 逻辑定义：0 代表无药(空载)，1 代表有药(遮挡)。
     * ==================================================================== */
    // 读取真实物理电平（假设低电平 0 为被遮挡有药，高电平 1 为无药空载）
    bool raw_has_medicine = (DL_GPIO_readPins(GPIOB, GPIO_SENSOR_PIN_LOAD_PHOTO_PIN) == 0);
    
    static uint8_t medicine_history = 0x00;
    static bool is_first_run = true;
    //--------------载药逻辑
    if (is_first_run || s_Reset_Debounce_Flag) {
        // 开机首帧，用真实物理状态填满整个历史池，防止虚假边沿
        medicine_history = raw_has_medicine ? 0xFF : 0x00;
        is_first_run = false;
s_Reset_Debounce_Flag = false; // 【关键】重置完成，消费掉标志位，防止死锁！
    } else {
        // 每次 10ms，将历史数据左移 1 位，并将最新状态推入最低位
        medicine_history = (medicine_history << 1) | (raw_has_medicine ? 0x01 : 0x00);
    }

    // 【算法升级】：位掩码 0x0F (二进制 1111) 提取最后 4 帧的时间切片
    // 0x03 (二进制 0011): 之前稳定没药2帧，现在稳定有药2帧 -> 【触发放药动作】
    //action_put_medicine  = 1;//test
    action_put_medicine  = ((medicine_history & 0x0F) == 0x03); //0x03
    /* [2026-06-09 诊断] 黄灯闪一次确认传感器上升沿被正确捕获 */
    if (action_put_medicine) {
        DL_GPIO_togglePins(GPIOB, GPIO_SENSOR_PIN_LED_Y_PIN);
    }
    // 0x0C (二进制 1100): 之前稳定有药2帧，现在稳定没药2帧 -> 【触发取药动作】
    bool action_take_medicine = ((medicine_history & 0x0F) == 0x0C);//0x0C
    //--------------载药逻辑

    static float cur_base_v = 0.0f;
    static float cur_vision_polarity = 1.0f;
    //Control_Mode_t cur_outer_mode = MODE_STRAIGHT_LINE;
// 【新增】：根据状态动态选择权重
    static float cur_weight_v = 0.7;
    static float cur_weight_g = 0.3;
                    static bool s_has_passed_intersection = false;

    /* ================================================================
     * 【漂移补偿预备】: 检测状态变迁, 在进入 IDLE / BRAKE 时设首次标志
     * ================================================================ */
    {
        static SystemState_t s_prev_state = STATE_IDLE;
        if (s_current_state != s_prev_state) {
            if (s_current_state == STATE_IDLE) {
                s_idle_first_entry = true;   /* 下次 IDLE 迭代重新捕获参考偏角 */
            }
            if (s_current_state == STATE_BRAKE_PHARMACY) {
                s_brake_first_entry = true;  /* 下次 BRAKE 迭代锁存停车偏角 */
            }
            /* [2026-05-28] WAIT_INTERSECTION 也加入漂移补偿检测链 */
            if (s_current_state == STATE_WAIT_INTERSECTION) {
                s_wait_first_entry = true;   /* 下次 WAIT 迭代重新捕获参考偏角 */
            }
            s_prev_state = s_current_state;
        }
    }

    // 请根据您的实际模块是常高还是常低调整 '==' 后面的逻辑
    switch (s_current_state) {
        case STATE_IDLE:
            cur_base_v = 0.0f;
            cur_weight_v = 0.0f; cur_weight_g = 1.0f;
            s_yaw_error = 0.0f;   /* [2026-06-01] IDLE时清零遥测脏值 */
// 🌟 引入局部记忆标志位，解耦指令和动作
            static bool wait_for_medicine = false;
            if (cmd == 'G') {
                wait_for_medicine = true; 
            }

            // 发车条件：收到树莓派 'G' 指令 或者 连续 4帧稳定检测到【放上药品】
            if ((wait_for_medicine && action_put_medicine)|| g_Coop_Remote_Go_Flag) { 
                s_current_state = STATE_FORWARD_TRACK;
                s_has_passed_intersection = false;
wait_for_medicine = false;     // 清除等待标志
                g_Coop_Remote_Go_Flag = false;
                                g_Yaw_Offset = yaw;
                                s_target_yaw_abs = 0.0f;
                                s_last_cardinal_target = 0.0f; /* 发车时绝对基准归零 */
                /* 离开 IDLE, 下次再次进入时重新捕获参考偏角 */
                s_idle_first_entry = true;
            }

            /* ================================================================
             * 【陀螺仪漂移补偿】: IDLE 状态下持续检测零漂
             * 原理 — MPU6050 DMP 的角度测量存在随时间累积的误差,
             *        即使车体静止, 反馈角度也会缓慢漂移
             * 策略 — 以进入 IDLE 瞬间的相对偏角为参考零点,
             *        若后续测量偏离 >= 1°, 平移 g_Yaw_Offset 拉回零点
             * ================================================================ */
            {
                float current_yaw_rel = GET_RELATIVE_YAW();
                if (s_idle_first_entry) {
                    /* 首次进入 IDLE: 记录当前偏角为参考零点 */
                    s_idle_entry_yaw = current_yaw_rel;
                    s_idle_first_entry = false;
                } else {
                    float drift = current_yaw_rel - s_idle_entry_yaw;
                    if (fabs(drift) >= 1.0f) {
                        /* 漂移 >= 1°: 平移偏置消除累积误差 */
                        g_Yaw_Offset += drift;
                        s_idle_entry_yaw = GET_RELATIVE_YAW();
                    }
                }
            }

            /* [光电归零]: 对管电平跳变时(取/放药品), 车体可能因物理触碰产生微动,
             * 立即将当前航向校准为新零点, 确保发车时角度基准准确 */
            if (action_put_medicine || action_take_medicine) {
                g_Yaw_Offset = yaw;
                s_target_yaw_abs = 0.0f;
                s_idle_entry_yaw = GET_RELATIVE_YAW();
            }
            break;
        case STATE_FORWARD_TRACK:
            cur_base_v = g_FSM_Base_Speed;
            cur_vision_polarity = 1.0f;
            /* [2026-06-11] 转弯后500ms陀螺仪锁航过渡(同AH/DH), 之后纯视觉 */
            if (s_post_turn_hold > 0) {
                cur_weight_v = 0.0f; cur_weight_g = 1.0f;
                s_post_turn_hold--;
            } else {
                cur_weight_v = 1.0f; cur_weight_g = 0.0f;
            }
// 【路口触发】：收到 S，立即进入路口挂起态
            if (cmd== 'S') {
                /* [非阻塞刹车]: 替代 delay_cycles(7200000)=225ms
                 * 30 周期 × 10ms ≈ 300ms 物理制动, DMP 每周期持续读取不溢出 */
                Motor_SetPWM(0, 0);          /* 左轮立即制动 */
                // Motor_SetPWM(1, 0);       /* [2026-05-28] 右轮延时刹车: 改为延时制动 */
                PID_Clear(&pid_speed_L);
                PID_Clear(&pid_speed_R);
                s_current_state = STATE_WAIT_INTERSECTION;
                // s_intersection_cooldown = 5; /* 旧硬编码 */
                // s_brake_timer  = 30;        /* 旧硬编码 */
                s_intersection_cooldown = g_FSM_Cooldown_Cycles;
                s_brake_timer  = g_FSM_Brake_Cycles;
                s_brake_action = BRAKE_ACTION_NONE;
                /* [2026-05-28] 装载右轮延时: 左轮机械制动慢, 右轮延迟以平衡车头 */
                s_right_brake_delay_timer = g_FSM_Right_Brake_Delay;
            }
            // 【药房触发】：收到 P，进入预停止缓冲态 (半速短冲 → 急刹)
            else if (cmd== 'P') {
                /* 预停止: 半速行驶 s_trans_fwd_timer 周期后自动切入 BRAKE_PHARMACY
                 * 非阻塞, 不刹车, 靠降速自然衰减惯性 */
                s_current_state = STATE_BRAKE_PHARMACY;
                Motor_SetPWM(0, 0);          /* 左轮立即制动 */
                // Motor_SetPWM(1, 0);       /* [2026-05-28] 右轮延时: 改为延时制动 */
                PID_Clear(&pid_speed_L);
                PID_Clear(&pid_speed_R);
                PID_Clear(&pid_angle_Z);
                PID_Clear(&pid_vision);
                /* [2026-05-28] 装载右轮延时刹车 */
                s_right_brake_delay_timer = g_FSM_Right_Brake_Delay;

                return;
                //s_current_state = STATE_PRE_STOP_PHARMACY;
                //s_trans_prestop_timer = g_FSM_Pre_Stop_Delay; /* 20 ×10ms = 200ms */
            }
            break;                    
        case STATE_WAIT_INTERSECTION:
            /* ================================================================
             * [2026-05-28] WAIT 漂移补偿: 与 IDLE/BRAKE 一致的静置校准
             * 路口停车等待转向指令期间, DMP 持续漂移
             * 漂移累积到 s_wait_drift_bias, 转向触发时注入 s_target_yaw_abs
             * g_Yaw_Offset 保持不变, 不触动绝对基准
             * ================================================================ */
            {
                /* [2026-06-10 简化] s_wait_entry_yaw 在进入 WAIT 时冻结为绝对基准,
                 * 不再更新(旧 ratcheting 每1°更新导致基准漂移 → 目标污染 → 多转)
                 * 总漂移 = 实时偏角 - 冻结基准, 由 PID 自然补偿 (target固定, actual含漂移) */
                float current_yaw_rel = GET_RELATIVE_YAW();
                if (s_wait_first_entry) {
                    s_wait_entry_yaw = current_yaw_rel;
                    s_wait_drift_bias = 0.0f;
                    s_wait_first_entry = false;
                } else {
                    s_wait_drift_bias = current_yaw_rel - s_wait_entry_yaw;
                }
            }
            /* [刹车冷却]: 计数未归零前无视一切A/D/W, 保证500ms物理停稳 */
            ///*
            if (s_intersection_cooldown > 0) {
                s_intersection_cooldown--;
                /* [2026-05-28] 冷却期间缓存 A/D/W, 冷却结束后消费 */
                if (cmd == 'A' || cmd == 'D' || cmd == 'W') {
                    s_cached_turn_cmd = cmd;
                }
                break;
            }
            /* [2026-05-28] 冷却结束: 消费缓存命令 */
            if (cmd == 0 && s_cached_turn_cmd != 0) {
                cmd = s_cached_turn_cmd;
                s_cached_turn_cmd = 0;
            }
            //*/
            cur_base_v = 0.0f;
            cur_weight_v = 0.0f; cur_weight_g = 0.0f;  /* 盲区: 原地死等 */
            if (cmd== 'W') {
                if (s_intersection_cooldown == 0) {
                    s_current_state = STATE_FORWARD_TRACK; /* 回到直行 */
                    /* 【状态切换补丁】: 本周期覆写为 FORWARD_TRACK 的运动学参数 */
                    cur_base_v = g_FSM_Base_Speed;
                    cur_vision_polarity = 1.0f;
                    cur_weight_v = 1.0f; cur_weight_g = 0.0f;
                 }

                //cmd= 0;
            } else if (cmd== 'A' || cmd== 'D') {
                if (s_intersection_cooldown == 0) {
                // 缓存转向意图，切入 0.1m/s 过渡态
                s_pending_turn_angle = (cmd== 'A') ? 90.0f : -90.0f;
// 静态标志位：用于区分小车是“直行来路口”还是“倒车回路口”

                if (s_has_passed_intersection) {
                    FSM_Trigger_Turn(STATE_TRANSITION_REV, s_pending_turn_angle);
                    s_current_state = STATE_TRANSITION_REV;
                    cur_base_v = g_FSM_Base_Speed;
                    // cur_base_v = 0.1f;  /* 旧硬编码, 改用全局变量统一 */
                    cur_weight_v = 0.0f; cur_weight_g = 1.0f;
                } else {
                    FSM_Trigger_Turn(STATE_TRANSITION_FWD, s_pending_turn_angle);
                    s_current_state = STATE_TRANSITION_FWD;
                    s_has_passed_intersection = true;
                    cur_base_v = g_FSM_Base_Speed;
                    // cur_base_v = 0.1f;  /* 旧硬编码, 改用全局变量统一 */
                    cur_weight_v = 0.0f; cur_weight_g = 1.0f;
                }
                //cmd= 0;
                } /* end if cooldown==0 */
            }
            break;
         case STATE_TRANSITION_FWD:
// 【微调】：速度减，关闭视觉，仅靠陀螺仪锁航向
            cur_base_v =g_FSM_Base_Speed;   // 【极低速盲走】：0.08m/s 走到几何中心
            cur_weight_v = 0.0f; cur_weight_g = 1.0f;  // 盲区
            if (s_trans_fwd_timer > 0) {
                s_trans_fwd_timer--;
            } else if (s_trans_entry_flag) {
                /* 正常过渡结束 (FSM_Trigger_Turn 已装载计时器): 刹车 → TURN_ABS */
                s_trans_entry_flag = false;
                /* [2026-05-28] 过渡结束时清零对方计时器, 杜绝跨状态残留 */
                s_trans_rev_timer = 0;
                s_trans_prestop_timer = 0;
                Motor_SetPWM(0, 0);
                Motor_SetPWM(1, 0);
                PID_Clear(&pid_speed_L);
                PID_Clear(&pid_speed_R);
                // s_brake_timer  = 30;                 /* 旧硬编码 */
                s_brake_timer  = g_FSM_Brake_Cycles;   /* 改用全局变量, 统一刹车时长 */
                s_brake_action = BRAKE_ACTION_TO_TURN_ABS;
                /* s_target_yaw_abs 累加由刹车到期回调完成 */
            } else {
                /* 异常入口 (timer=0 且非 Trigger 调用): 补装计时器, 防止跳过过渡 */
                s_trans_fwd_timer = g_FSM_Trans_Fwd_Delay - 1;
            }
                    break;
        case STATE_TRANSITION_REV:
        // 【修订】：触发即刹车逻辑已在 Trigger 中处理，此处执行 1/2 倍反向回归
            cur_base_v = g_FSM_Base_Speed;
            cur_weight_v = 0.0f; cur_weight_g = 1.0f;  // 盲区
            if (s_trans_rev_timer > 0) {
                s_trans_rev_timer--;
            } else if (s_trans_entry_flag) {
                /* 正常过渡结束 (FSM_Trigger_Turn 已装载计时器): 刹车 → TURN_ABS */
                s_trans_entry_flag = false;
                /* [2026-05-28] 过渡结束时清零对方计时器, 杜绝跨状态残留 */
                s_trans_fwd_timer = 0;
                s_trans_prestop_timer = 0;
                Motor_SetPWM(0, 0);
                Motor_SetPWM(1, 0);
                PID_Clear(&pid_speed_L);
                PID_Clear(&pid_speed_R);
                // s_brake_timer  = 30;                 /* 旧硬编码 */
                s_brake_timer  = g_FSM_Brake_Cycles;   /* 改用全局变量, 统一刹车时长 */
                s_brake_action = BRAKE_ACTION_TO_TURN_ABS;
                /* s_target_yaw_abs 累加由刹车到期回调完成 */
            } else {
                /* 异常入口 (timer=0 且非 Trigger 调用): 补装计时器, 防止跳过过渡 */
                s_trans_rev_timer = g_FSM_Trans_Rev_Delay - 1;
            }
            break;

        case STATE_TURN_ABS:
            cur_base_v = 0.0f;
	cur_weight_v = 0.0f; cur_weight_g = 1.0f;  /* 盲区 */
            /* [2026-06-01] PD方向: Gyro_Only_Control 内部用 Normalize_Angle 定最短路径
             * 完成判定: 连续 yaw 差 → 补偿 DMP 高速旋转少报角度 */
            {
                /* [2026-06-11] 完全移植调试层AH/DH: 到达目标立即切,
                 * 不在TURN_ABS内锁航. 锁航由FORWARD_TRACK的陀螺仪过渡期负责 */
                float raw_err = Normalize_Angle(s_target_yaw_abs - GET_RELATIVE_YAW());

                if (fabs(raw_err) <= g_FSM_Turn_Tolerance) {
                    if (s_notify_pi_turn_done) {
                        /* 药房旧180°(备用): CTM→刹车3s→循迹 */
                        DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'C');
                        DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'T');
                        DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'M');
                        PID_Clear(&pid_speed_L);  PID_Clear(&pid_speed_R);
                        Motor_SetPWM(0, 0); Motor_SetPWM(1, 0);
                        s_brake_timer  = 300;
                        s_brake_action = BRAKE_ACTION_TO_FORWARD_TRACK;
                    } else if (s_pharmacy_turns_left > 0) {
                        s_pharmacy_turns_left--;
                        if (s_pharmacy_turns_left > 0) {
                            /* 第二次D: 以走廊航向180目标算剩余, 自动补偿第一次误差 */
                            float total_target = s_pharmacy_ref_yaw - 180.0f;
                            float remaining = Normalize_Angle(total_target - GET_RELATIVE_YAW());
                            s_turn_target_override = total_target;
                            s_turn_target_override_valid = true;
                            FSM_Trigger_Turn(STATE_TURN_ABS, remaining);
                        } else {
                            /* 2次D都完成: 立即CTM→刹车3s→FORWARD_TRACK */
                            DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'C');
                            DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'T');
                            DL_UART_Main_transmitDataBlocking(UART_PI_INST, 'M');
                            /* [2026-06-20] split-180 已在此处发送CTM。
                             * 不置 s_notify_pi_turn_done, 否则若后续复检重入 TURN_ABS,
                             * 会误走旧180备用分支并再次追加300拍停车。
                             */
                            s_notify_pi_turn_done = false;
                            PID_Clear(&pid_speed_L);  PID_Clear(&pid_speed_R);
                            Motor_SetPWM(0, 0); Motor_SetPWM(1, 0);
                            /* [2026-06-12] Pharmacy return after 2x90 turn:
                             * old value 300(3s) was too long after the
                             * split-180 logic became stable.  Use half delay
                             * so the return leg starts sooner; keep this
                             * local to the BRAKE_PHARMACY-origin path.
                             */
                            s_brake_timer  = 150; /* was 300 */
                            s_brake_action = BRAKE_ACTION_TO_FORWARD_TRACK;
                        }
                    } else {
                        /* [2026-06-12 A/D过冲修复]
                         * 调试层 fsm_debug.c 的 STATE_TURN_ABS 在误差进容差后立即切回
                         * STATE_FORWARD_TRACK，不额外等待角速度降到阈值，也不追加300ms刹车。
                         * 主层原逻辑会在目标附近继续用 PD 反向压角速度，落地摩擦和电机响应差异
                         * 会把这段“主动压速”变成概率性回摆。普通90度转弯在这里直接进入前进态，
                         * 并保留 s_target_yaw_abs 作为500ms陀螺仪锁航目标，负责消化剩余小误差。
                         */
                        s_post_turn_hold = 50;
                        s_current_state = STATE_FORWARD_TRACK;
                        s_brake_action = BRAKE_ACTION_NONE;
                        cur_base_v = g_FSM_Base_Speed;
                        cur_weight_v = 0.0f;
                        cur_weight_g = 1.0f;
                        PID_Clear(&pid_speed_L);
                        PID_Clear(&pid_speed_R);
                        PID_Clear(&pid_angle_Z);
                        PID_Clear(&pid_vision);
                        break;
                    }
                    return;
                }
            }
            break;
        case STATE_PRE_STOP_PHARMACY:
            cur_base_v = 0.5 * g_FSM_Base_Speed; // 
            cur_weight_v = 0.0f; cur_weight_g = 1.0f;            // 盲区：屏蔽药房标识干扰
            if (s_trans_prestop_timer > 0) {
                s_trans_prestop_timer--;
            } else {
                s_current_state = STATE_BRAKE_PHARMACY; // 时间到，彻底切入急刹
            }
            break;
        case STATE_BRAKE_PHARMACY:
            cur_base_v = 0.0f; // 停在药房死锁不动
            cur_weight_v = 0.0f; cur_weight_g = 1.0f;  // 盲区
            Motor_SetPWM(0, 0);Motor_SetPWM(1, 0);

            /* ================================================================
             * 【陀螺仪漂移补偿 (停车态) — 独立偏置量, 不触动绝对基准】
             * 发车后 g_Yaw_Offset 是绝对基准, 不可修改
             * BRAKE 期间 DMP 零漂累积到 s_brake_drift_bias
             * 进入反向循迹时一次性注入 s_target_yaw_abs, 然后清零
             * 这样 GET_RELATIVE_YAW() 的漂移被偏置抵消, 误差归零
             * ================================================================ */
            {
                /* [2026-06-10 简化] s_brake_entry_yaw 药房停车时冻结为绝对基准,
                 * 不再更新(旧 ratcheting 每1°更新导致基准漂移 → 180°目标污染)
                 * 总漂移 = 实时偏角 - 冻结基准, 由 PID 自然补偿 */
                float current_yaw_rel = GET_RELATIVE_YAW();
                if (s_brake_first_entry) {
                    s_brake_entry_yaw = current_yaw_rel;
                    s_brake_drift_bias = 0.0f;
                    s_brake_first_entry = false;
                } else {
                    s_brake_drift_bias = current_yaw_rel - s_brake_entry_yaw;
                }
            }

            /* ================================================================
             * [2026-05-31 修订] 药房出发: 原地180°转弯 → 正向循迹
             * 旧方案: FSM_Trigger_Reverse_Start() → STATE_REVERSE_TRACK (倒车循迹)
             * 修订原因: 倒车循迹受视觉盲区+陀螺漂移双重影响, 稳定性远不如正向循迹
             * [2026-06-10 重构] target = s_brake_entry_yaw(冻结) + 180°
             * 漂移由PID自然补偿, 不手动注入偏置
             * 链路: 刹车 → BRAKE_ACTION_TO_TURN_ABS → TURN_ABS(原地转180°)
             *       → BRAKE_ACTION_TO_FORWARD_TRACK → STATE_FORWARD_TRACK
             * ================================================================ */
            // 无论是小车1去送药（等取走），还是小车2去取药（等放上）
            // 只要发生稳定的【动作边沿跳变】，立刻原地180°掉头 → 正向循迹返回！
            if (action_put_medicine || action_take_medicine) {
                Coop_Send_Start_Cmd(g_Target_Room);
                /* 离开 BRAKE, 下次再次进入时重新锁存停车偏角 */
                s_brake_first_entry = true;

                /* [2026-05-31] 旧代码搁置 (逻辑上失能, 保留代码供参考):
                 * FSM_Trigger_Reverse_Start();  // → STATE_REVERSE_TRACK 倒车循迹
                 * 替换为下方原地180°转弯逻辑 */

                /* [2026-06-20] 180°=2×D, 但总目标基准取视觉校准航向。
                 * 若车在药房因视觉PD停歪, GET_RELATIVE_YAW() 已包含偏角；
                 * s_target_yaw_abs 是进入药房前由红线视觉校准出的走廊方向。
                 * 因此用 s_target_yaw_abs - 180° 作为总目标, 才能回程摆正。
                 */
                s_pharmacy_ref_yaw = s_target_yaw_abs;
                s_pharmacy_turns_left = 2;
                s_turn_target_override = s_pharmacy_ref_yaw - 90.0f;
                s_turn_target_override_valid = true;
                FSM_Trigger_Turn(STATE_TURN_ABS, -90.0f);
            }
            break;

        /* [2026-05-31 失能] STATE_REVERSE_TRACK: 反向循迹 (药房→十字路口)
         * 旧入口: FSM_Trigger_Reverse_Start() 已失能, 本状态不再可达
         * 替代方案: BRAKE_PHARMACY → 原地180°转弯 → STATE_FORWARD_TRACK
         * 保留全部代码供参考, 若未来需恢复倒车循迹, 取消 FSM_Trigger_Reverse_Start 的失能标记即可 */
        case STATE_REVERSE_TRACK:
            cur_base_v = g_FSM_Reverse_Speed;
            cur_vision_polarity = -1.0f;
            /* 【纯陀螺仪循迹】: 反向循迹 (药房→第一个十字路口) 仅依赖陀螺仪
             * 视觉完全屏蔽, 使用独立 PID 结构体 pid_angle_Z_rev
             * 参数在 FSM_Trigger_Reverse_Start() 中通过 FSM_Apply_Reverse_Gyro_PID() 装载
             * 与转弯90度模态的 pid_angle_Z 完全解耦 */
            cur_weight_v = 0.0f;
            cur_weight_g = 1.0f;
            // 倒车时收到树莓派转向指令，触发”倒车专属过渡态”
            // 必须在这里捕捉它，并切入路口等待态，否则车会无视路口直接撞墙！
            if (cmd== 'S') {
                /* [一脚杀死]: 短路刹车 + 500ms冷却锁 */
                Motor_SetPWM(0, 0);          /* 左轮立即制动 */
                // Motor_SetPWM(1, 0);       /* [2026-05-28] 右轮延时: 改为延时制动 */
                PID_Clear(&pid_speed_L);
                PID_Clear(&pid_speed_R);
                s_current_state = STATE_WAIT_INTERSECTION;
                // s_intersection_cooldown = 5; /* 旧硬编码 */
                // s_brake_timer  = 30;        /* 旧硬编码 */
                s_intersection_cooldown = g_FSM_Cooldown_Cycles;
                s_brake_timer  = g_FSM_Brake_Cycles;
                s_brake_action = BRAKE_ACTION_NONE;
                /* [2026-05-28] 装载右轮延时刹车 */
                s_right_brake_delay_timer = g_FSM_Right_Brake_Delay;

                //cmd= 0;
            }
            /*
            if (cmd== 'A') { static uint32_t ts3=0;if(!ts3++){FSM_Trigger_Turn(STATE_TRANSITION_REV, 90.0f);} //cmd= 0;
            }
            if (cmd== 'D') {  static uint32_t  ts4=0;if(!ts4++){FSM_Trigger_Turn(STATE_TRANSITION_REV, -90.0f);}// cmd= 0; 
            }
            */
            break;
        default: break;  /* ANGLE_HOLD/TURN_HOLD 未使用 */
    }
/* ====================================================================
     * 【核心注入】：物理急刹车与防抽搐拦截 (必须放在 PID 计算之前！)
     * ==================================================================== */
    // 【附带红利】：顺手把 STATE_BRAKE_PHARMACY (药房停靠) 的刹车抽搐也一并修了！
if (s_current_state == STATE_IDLE || 
        s_current_state == STATE_BRAKE_PHARMACY || 
        s_current_state == STATE_WAIT_INTERSECTION) {
        
        Motor_SetPWM(0, 0); 
        Motor_SetPWM(1, 0);
        PID_Clear(&pid_speed_L);
        PID_Clear(&pid_speed_R);
        g_Target_Pulse_L = 0;
        g_Target_Pulse_R = 0;
        /*
        if(g_Pi_Cmd!=last_g_Pi_Cmd){
        if(stt<32){
        hst[stt++]=g_Pi_Cmd;
        }else{stt=0;hst[stt++]=g_Pi_Cmd;}
    }
*/
        return; // 直接拔掉钥匙，跳过下面的多速率矩阵与内环解算！
    }
    // 🛡️ yaw 经 mymain.c 解卷绕后为连续角度, Normalize_Angle 仅对误差取最短路径
    //     g_Gyro_Z_Rate 由解卷绕差分算出, 供 PD 外环 D 项物理阻尼
    float current_yaw = (yaw - g_Yaw_Offset);
    // 1. 计算真正物理意义上的最短偏转误差 (例如: 180 - (-179.9) = 359.9 -> 归一化为 -0.1)
    float true_yaw_error = Normalize_Angle(
        s_target_yaw_abs - current_yaw);
    s_yaw_error = true_yaw_error;
    float aligned_target_yaw = current_yaw + true_yaw_error;

    /* 根据当前状态路由到正确的控制函数 */
    float outer_comp;
    if (s_current_state == STATE_FORWARD_TRACK) {
        /* [2026-06-11] 按权重路由: 转弯后陀螺仪锁航过渡→纯视觉循迹 */
        if (cur_weight_g > 0.001f && cur_weight_v > 0.001f) {
            outer_comp = Fusion_Outer_Loop(current_vision_err, aligned_target_yaw,
                           current_yaw, g_Gyro_Z_Rate, cur_vision_polarity,
                           cur_weight_v, cur_weight_g);
        } else if (cur_weight_g > 0.001f) {
            outer_comp = Gyro_Only_Control(aligned_target_yaw, current_yaw,
                                           g_Gyro_Z_Rate, &pid_angle_Z);
        } else {
            outer_comp = Vision_Only_Control(current_vision_err, cur_vision_polarity);
        }
    } else if (s_current_state == STATE_REVERSE_TRACK) {
        /* 反向循迹: 纯陀螺仪航向锁定, 使用独立 PID 结构体 pid_angle_Z_rev
         * 参数由 FSM_Apply_Reverse_Gyro_PID() 在进入反向循迹时装载
         * 与转弯90度模态的 pid_angle_Z 完全解耦, 互不干扰 */
        outer_comp = Gyro_Only_Control(aligned_target_yaw, current_yaw,
                                       g_Gyro_Z_Rate, &pid_angle_Z_rev);
    } else {
        /* 转弯/过渡态 (TRANSITION_FWD / TRANSITION_REV / TURN_ABS / PRE_STOP):
         * 纯陀螺仪航向锁定, 使用转弯专用 PID 结构体 pid_angle_Z
         * 参数在 mymain.c 中初始化, 通过 VOFA 的 PG=/DG= 在线可调
         * 与调试层 (fsm_debug.c) 共用同一 PID 实体, 确保转弯性能可复现 */
        outer_comp = Gyro_Only_Control(aligned_target_yaw, current_yaw,
                                       g_Gyro_Z_Rate, &pid_angle_Z);
    }
    Kinematics_Update_Target(cur_base_v, outer_comp);

    out_L = PID_Calc_PI_Inner(&pid_speed_L, g_Target_Pulse_L, (float)g_Chassis.speed_L);
    out_R = PID_Calc_PI_Inner(&pid_speed_R, g_Target_Pulse_R, (float)g_Chassis.speed_R);

    Motor_SetPWM(0, (int16_t)out_L); Motor_SetPWM(1, (int16_t)out_R);

}
