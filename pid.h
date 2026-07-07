#ifndef PID_H
#define PID_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// 物理参数
#define PULSES_PER_REV      (390.0f) 
#define WHEEL_DIAMETER_M    (0.066f)                 
#define WHEEL_PERIMETER_M   (WHEEL_DIAMETER_M * 3.14159265f) 
#define CONTROL_PERIOD_S    (0.01f) 
#define PULSE_TO_MS_COEF    ((100.0f * WHEEL_PERIMETER_M)/PULSES_PER_REV)
// 100ms 周期专用的系数 (乘以 10 即可换算为 1 秒)
// 核心结构体
typedef struct {
    float Kp; float Ki; float Kd;
    float target; float actual;
    float err; float err_last; float integral;
    float out; float out_max; float integral_max;
    float deadband; float i_band;
} PID_TypeDef;



// 【全量 Extern 声明：实体变量】
extern PID_TypeDef pid_speed_L;
extern PID_TypeDef pid_speed_R;
extern PID_TypeDef pid_angle_Z;
extern PID_TypeDef pid_vision;
extern PID_TypeDef pid_angle_Z_rev;  /* 反向循迹专用陀螺仪PD环, 参数独立于转弯90度PD */

extern float alp;
extern float g_Gyro_Z_Rate;
extern float Normalize_Angle(float angle);

extern float g_Target_Pulse_L;
extern float g_Target_Pulse_R;
extern float g_Weight_Vision_Fwd; // 正向视觉权重
extern float g_Weight_Vision_Rev; // 倒退视觉权重
extern float g_Weight_Gyro_Fwd;   // 正向陀螺仪阻尼权重
extern float g_Weight_Gyro_Rev;   // 倒退陀螺仪阻尼权重

/* 反向循迹独立陀螺仪PD参数 (VOFA可调) */
extern float g_Rev_Gyro_Kp;
extern float g_Rev_Gyro_Kd;

// 【全量 Extern 声明：操作算子】
extern void PID_Init(PID_TypeDef *pid, float p, float i, float d, float out_max, float i_max, float deadband, float i_band);
extern void PID_Clear(PID_TypeDef *pid);
extern float PID_Calc_PI_Inner(PID_TypeDef *pid, float target, float actual);
extern float PID_Calc_PD_Outer_With_Rate(PID_TypeDef *pid, float target_angle, float actual_angle, float actual_rate);
extern float PID_Calc_PD_Outer_Discrete(PID_TypeDef *pid, float target_offset, float actual_offset);
extern float Convert_Velocity_To_Pulses(float target_velocity_m_s);
extern void Kinematics_Update_Target(float base_v_m_s, float turn_compensation);
extern float Fusion_Outer_Loop(float vision_offset, float target_yaw,
                               float gyro_yaw, float gyro_z_rate, float vision_polarity,
                               float current_weight_v, float current_weight_g);

/* 纯陀螺仪航向锁定控制 (反向循迹/转弯模态使用, 可传入独立PID结构体) */
extern float Gyro_Only_Control(float target_yaw, float current_yaw,
                               float gyro_z_rate, PID_TypeDef *pid);

/* 纯视觉循迹控制 (正向循迹使用) */
extern float Vision_Only_Control(float vision_offset, float vision_polarity);

#endif