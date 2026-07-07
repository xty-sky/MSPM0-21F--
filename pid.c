#include "pid.h"
#include "bsp_mpu6050.h"
#include <math.h>

// 角度归一化算子 (消除 ±180° wrap 对差值计算的影响)
float Normalize_Angle(float angle) {
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

PID_TypeDef pid_speed_L;
PID_TypeDef pid_speed_R;
PID_TypeDef pid_angle_Z;
PID_TypeDef pid_vision;
PID_TypeDef pid_angle_Z_rev;    /* 反向循迹专用陀螺仪PD环实体 */
// 预留给外部引用的双轮解算目标 (单位：脉冲/10ms)
float g_Target_Pulse_L = 0.0f;
float g_Target_Pulse_R = 0.0f;
// 陀螺仪 Z 轴角速率 (deg/s)，供 PD 外环 D 项阻尼
float g_Gyro_Z_Rate = 0.0f;
// 融合权重系数 (可在 VOFA 中在线整定)_直行时视觉与航向的影响配比
float g_Weight_Vision_Fwd = 0.7f; 
float g_Weight_Gyro_Fwd   = 0.3f; 
float g_Weight_Vision_Rev = 0.2f; // 倒车视觉权重必须极低！防甩尾
float g_Weight_Gyro_Rev   = 0.8f; // 倒车高度依赖陀螺仪锁死姿态
float alp                = 0.8f;

/* 反向循迹独立陀螺仪PD参数 (与转弯90度PID解耦, 可VOFA在线调整)
 * 默认值与转弯 pid_angle_Z 一致 (Kp=0.13, Kd=0.015), 废弃旧假设 (0.095/0.1)
 * 实际运行时由 FSM_Apply_Reverse_Gyro_PID() 装载至 pid_angle_Z_rev */
float g_Rev_Gyro_Kp = 0.13f;    /* 反向循迹陀螺仪PD: 比例系数 */
float g_Rev_Gyro_Kd = 0.015f;   /* 反向循迹陀螺仪PD: 微分系数 */

/* ====================================================================
 * 操作单元：PID 参数装载与初始化
 * ==================================================================== */
void PID_Init(PID_TypeDef *pid, float p, float i, float d, float out_max, float i_max, float deadband, float i_band) {
    pid->Kp = p;  pid->Ki = i;  pid->Kd = d;
    pid->out_max = out_max;
    pid->integral_max = i_max;
    pid->deadband = deadband;
    pid->i_band = i_band;
    PID_Clear(pid);
}

void PID_Clear(PID_TypeDef *pid) {
    pid->target = 0.0f;  pid->actual = 0.0f;
    pid->err = 0.0f;     pid->err_last = 0.0f;
    pid->integral = 0.0f; pid->out = 0.0f;
}

/* ====================================================================
 * 算子：内环速度环专用 (严格 PI 控制)
 * 机制：死区拦截 -> 积分清空(过大误差) -> PI 计算 -> 限幅
 * ==================================================================== */
float PID_Calc_PI_Inner(PID_TypeDef *pid, float target, float actual) {
    pid->target = target;
    pid->actual = actual;
    pid->err = pid->target - pid->actual;
    
    // 1. 防高频微颤：死区
    if (fabs(pid->err) <= pid->deadband) {
        pid->err = 0.0f;
    }
    
    // 2. 积分清零与抗饱和机制
    if (fabs(pid->err) > pid->i_band) {
        pid->integral = 0.0f; // 偏差过大时，强行切断积分，防止风饱和
    } else {
        pid->integral += pid->err;
        if (pid->integral > pid->integral_max)       pid->integral = pid->integral_max;
        else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    }
    
    // 3. 纯 PI 计算 (强制无视 Kd)
    pid->out = (pid->Kp * pid->err) + (pid->Ki * pid->integral);
               
    // 4. 限幅
    if (pid->out > pid->out_max)       pid->out = pid->out_max;
    else if (pid->out < -pid->out_max) pid->out = -pid->out_max;
    
    return pid->out;
}

/* ====================================================================
 * 外环算子 A：带物理真实导数 (陀螺仪) 的 PD 环
 * 物理依据：角度的导数就是角速度。直接代入角速度可彻底消除差分噪声！
 * ==================================================================== */
float PID_Calc_PD_Outer_With_Rate(PID_TypeDef *pid, float target_angle, float actual_angle, float actual_rate) {
    pid->target = target_angle;
    pid->actual = actual_angle;
    pid->err = target_angle - actual_angle;

    if (fabs(pid->err) <= pid->deadband) pid->err = 0.0f;

    pid->out = (pid->Kp * pid->err) - (pid->Kd * actual_rate);

    if (pid->out > pid->out_max)       pid->out = pid->out_max;
    else if (pid->out < -pid->out_max) pid->out = -pid->out_max;

    return pid->out;
}

/* ====================================================================
 * 外环算子 B：无物理导数 (树莓派视觉) 的离散 PD 环
 * ==================================================================== */
float PID_Calc_PD_Outer_Discrete(PID_TypeDef *pid, float target_offset, float actual_offset) {
    pid->err = target_offset - actual_offset;
    //static differ_last = 0, differ_now = 0;
    if (fabs(pid->err) <= pid->deadband) pid->err = 0.0f;
    //differ_now  = (pid->Kd * (pid->err - pid->err_last));
    //pid->out = (pid->Kp * pid->err) + differ_now + alp*differ_last;
    pid->out = (pid->Kp * pid->err) + (pid->Kd * (pid->err - pid->err_last));
    //differ_last  = differ_now;
    
    pid->err_last = pid->err;
    
    if (pid->out > pid->out_max)       pid->out = pid->out_max;
    else if (pid->out < -pid->out_max) pid->out = -pid->out_max;
    
    return pid->out;
}


/* ====================================================================
 * 【核心重构】：去耦合加权融合中枢
 * 对应 fsm.c 调用：Fusion_Outer_Loop(current_vision_err, s_target_yaw_abs, 
 * GET_RELATIVE_YAW(), 0.0f, vision_polarity, ...)
 * ==================================================================== */
float Fusion_Outer_Loop(float vision_offset, float target_yaw, 
                        float current_yaw, float gyro_z_rate, float vision_polarity,
                        float weight_v, float weight_g) 
{
     float vision_out, gyro_out = 0;
// 🛡️ 防御性隔离：权重为 0 时，不仅切断输出，更要冻结状态更新！
    if (weight_v > 0.001f) {
        vision_out = PID_Calc_PD_Outer_Discrete(&pid_vision, 0.0f, vision_offset * vision_polarity);
    } else {
        // 如果视觉被剥离，强制清空其历史包袱，防止切回时 D 项炸裂
        PID_Clear(&pid_vision); 
    }
    
    if (weight_g > 0.001f) {
        gyro_out = PID_Calc_PD_Outer_With_Rate(&pid_angle_Z, target_yaw, current_yaw, gyro_z_rate);
    } else {
        PID_Clear(&pid_angle_Z);
    }    
    // 2. 数学自洽融合：由权重直接决定模态
    // 当 FSM 传入 weight_v=0 时，vision_out 自动失效
    float final_out = (vision_out * weight_v) + (gyro_out * weight_g);
    
    // 3. 安全限幅 (复用陀螺仪环的满量程限制)
    if (final_out > pid_angle_Z.out_max)  final_out =  pid_angle_Z.out_max;
    if (final_out < -pid_angle_Z.out_max) final_out = -pid_angle_Z.out_max;
    
    return final_out;
}

/* ====================================================================
 * 算子：纯陀螺仪航向锁定控制
 * 用于反向循迹、90度转弯等纯依赖陀螺仪的模态
 * 参量 pid 可传入 &pid_angle_Z (转弯) 或 &pid_angle_Z_rev (反向循迹)
 * 实现与调试层 STATE_TURN_ABS 相同的计算链路
 * ==================================================================== */
float Gyro_Only_Control(float target_yaw, float current_yaw,
                        float gyro_z_rate, PID_TypeDef *pid) {
    /* 计算最短路径偏角误差, 对齐目标角 */
    float yaw_error_raw = target_yaw - current_yaw;
    float yaw_error_min = Normalize_Angle(yaw_error_raw);
    float aligned_target = current_yaw + yaw_error_min;
    /* 带真实角速度的 PD 环, D 项由陀螺仪硬件提供
     * PD输出与 Kinematics 约定一致: turn>0→左转(yaw↑), PD正误差→正输出→左转 */
    return PID_Calc_PD_Outer_With_Rate(pid, aligned_target, current_yaw, gyro_z_rate);
}

/* ====================================================================
 * 算子：纯视觉循迹控制
 * 用于正向循迹 (STATE_FORWARD_TRACK), 仅依赖树莓派视觉偏差
 * ==================================================================== */
float Vision_Only_Control(float vision_offset, float vision_polarity) {
    /* 目标偏移始终为 0 (摄像头中心对准红线), 实际偏移经极性修正 */
    return PID_Calc_PD_Outer_Discrete(&pid_vision, 0.0f,
                                      vision_offset * vision_polarity);
}

/* ====================================================================
 * 算子：直观物理速度转底层 10ms 脉冲差量
 * ==================================================================== */
float Convert_Velocity_To_Pulses(float target_velocity_m_s) {
    return target_velocity_m_s / PULSE_TO_MS_COEF;
}
/* ====================================================================
 * 算子：运动学解耦阵列 (直行与原地转弯的物理映射层)
 * 输入 base_v_m_s: 基础前进速度。如果为 0，则处于原地转弯工况。
 * 输入 turn_compensation: 外环 PD 计算出的转向差速补偿量。
 * 输出：直接写入全局变量 g_Target_Pulse_L 与 g_Target_Pulse_R
 * ==================================================================== */
void Kinematics_Update_Target(float base_v_m_s, float turn_compensation) {
    float base_pulses = Convert_Velocity_To_Pulses(base_v_m_s);
    // 左轮目标 = 基础速度 - 转向补偿
    g_Target_Pulse_L = base_pulses - turn_compensation;
    // 右轮目标 = 基础速度 + 转向补偿
    g_Target_Pulse_R = base_pulses + turn_compensation;
}