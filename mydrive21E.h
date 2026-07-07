// mydrive21F.h
//预编译防卫宏，防止该头文件被多处 include 时引发重复定义的编译错误
#ifndef MYDRIVE21E_H
#define MYDRIVE21E_H

// 引入 SysConfig 生成的底层硬件宏定义
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
//#include <stdmath.h>
/* ====================================================================
 * 宏定义：指示灯原子化操作算子 (依托 SysConfig 自动生成的底层宏)
 * 物理映射：红灯(PB26), 绿灯(PB27), 黄灯(PB25)
 * ==================================================================== */
/* ====================================================================
 * 底盘物理层极性标定算子 (Calibration Polarity)
 * 规则：1 为正向，-1 为反向。
 *
 * [2026-05-31 全量标定] 基于开环实验确定:
 *   Test: Motor_SetPWM(x, +300) → CCP3激活 → 后退
 *   → CCP2=前进, CCP3=后退
 *   前进: 左enc递增(delta>0), 右enc递减(delta<0)  [对侧安装]
 *   后退: 左enc递减(delta<0), 右enc递增(delta>0)
 *
 *   标定结果:
 *   MOTOR_L=1, MOTOR_R=1   → PID正输出→CCP2→前进
 *   ENC_L=1,   ENC_R=-1    → 前进时speed为正
 * ==================================================================== */
#define MOTOR_L_POLARITY  (1)    // [2026-05-31] 旧值(-1)→新值(1): PID正输出=前进
#define MOTOR_R_POLARITY  (1)    // [2026-05-31] 旧值(-1)→新值(1): PID正输出=前进

#define ENC_L_POLARITY    (1)    // [2026-05-31] 旧值(-1)→新值(1): 前进delta>0,需speed正
#define ENC_R_POLARITY    (-1)   // [2026-05-31] 旧值(1)→新值(-1): 前进delta<0,需speed正
// 点亮特定灯 (假设高电平点亮) - 【修改点】：将 GPIO_SENSOR_PORT 全部替换为 GPIOB
#define LED_RED_ON()      DL_GPIO_setPins(GPIOB, GPIO_SENSOR_PIN_LED_R_PIN)
#define LED_GREEN_ON()    DL_GPIO_setPins(GPIOB, GPIO_SENSOR_PIN_LED_G_PIN)
#define LED_YELLOW_ON()   DL_GPIO_setPins(GPIOB, GPIO_SENSOR_PIN_LED_Y_PIN)

// 熄灭特定灯
#define LED_RED_OFF()     DL_GPIO_clearPins(GPIOB, GPIO_SENSOR_PIN_LED_R_PIN)
#define LED_GREEN_OFF()   DL_GPIO_clearPins(GPIOB, GPIO_SENSOR_PIN_LED_G_PIN)
#define LED_YELLOW_OFF()  DL_GPIO_clearPins(GPIOB, GPIO_SENSOR_PIN_LED_Y_PIN)

// 熄灭所有灯 (利用按位或，1个CPU周期内同步执行)
#define LED_ALL_OFF()     DL_GPIO_clearPins(GPIOB, \
                            GPIO_SENSOR_PIN_LED_R_PIN | \
                            GPIO_SENSOR_PIN_LED_G_PIN | \
                            GPIO_SENSOR_PIN_LED_Y_PIN)
                            
// 物理约束：PWM 周期满量程 (对应 SysConfig 中的 Period Count)
#define PWM_PERIOD_MAX 1000

// 数据结构化：底盘运动状态矩阵
typedef struct {
    int32_t encoder_L_count;    // 左轮累计物理脉冲 (硬件 QEI)
    int32_t encoder_R_count;    // 右轮累计物理脉冲 (软件 EXTI)
    /* [2026-05-25 修复] last_enc 改为 int32_t, 与 encoder_count 类型一致,
     * 消除 uint16_t 截断 int32_t 的隐患 (累计脉冲超过 65535 时低 16 位回绕) */
    int32_t last_enc_L;          // 上一时间片左轮脉冲值
    int32_t last_enc_R;          // 上一时间片右轮脉冲值
    // uint16_t last_enc_L;      /* 旧类型, 已废弃 */
    // uint16_t last_enc_R;      /* 旧类型, 已废弃 */
    int16_t speed_L;            // 左轮瞬时速度 (脉冲/10ms)
    int16_t speed_R;            // 右轮瞬时速度 (脉冲/10ms)
} ChassisState_t;
/* ====================================================================
 * 自适应卡尔曼滤波器 (基于新息驱动)
 * 物理意义：R固定为1，Q根据测量误差的平方动态调整，实现“静如处子，动如脱兔”
 * ==================================================================== */
typedef struct {
    float x;            // 滤波后的最优估计速度
    float P;            // 估计协方差
    float noise_margin; // 编码器容忍的底噪阈值 (建议设为 2.0)
} AdaptiveKalman_t;

void AKF_Init(AdaptiveKalman_t *kf, float noise_margin);
float AKF_Update(AdaptiveKalman_t *kf, float measured_val);

extern AdaptiveKalman_t akf_speed_L;
extern AdaptiveKalman_t akf_speed_R;

// 外部全局变量声明 (告诉其他 .c 文件：这些变量实体在别处)
extern volatile ChassisState_t g_Chassis;
extern volatile bool g_Flag_10ms;
extern volatile uint32_t g_Control_Tick_Seq; /* SysTick每10ms递增, 用于DMP实际采样间隔估算 */

// 核心操作算子对外接口声明
void Motor_SetPWM(uint8_t motor_id, int16_t pwm_value);
void Update_Chassis_Speed(void);

#endif // MYDRIVE21E_H
