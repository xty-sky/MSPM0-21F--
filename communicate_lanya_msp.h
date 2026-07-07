#ifndef __COMMUNICATE_LANYA_MSP_H
#define __COMMUNICATE_LANYA_MSP_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"          /* SysConfig 生成的配置文件 */

/* ============================================================
 *   用户配置区 —— 根据你的 SysConfig 设置修改下列宏
 * ============================================================ */

/* UART 实例（SysConfig 中定义的名称，如 UART_0_INST） */
#define LANYA_UART_INST             UART_0_INST

/* UART 中断号（SysConfig 自动生成，如 UART_0_INST_INT_IRQN） */
#define LANYA_UART_IRQN             UART_0_INST_INT_IRQN

/* SET 引脚 GPIO 端口（SysConfig 中定义的 Port 宏） */
#define LANYA_SET_PORT              GPIOA

/* SET 引脚号（SysConfig 中定义的 Pin 宏，如 DL_GPIO_PIN_0） */
#define LANYA_SET_PIN               DL_GPIO_PIN_0

/* ============================================================
 *   以下不需要修改
 * ============================================================ */

/* 环形接收缓冲区大小 */
#define LANYA_RX_BUF_SIZE           256

/* E104 工作模式 */
#define LANYA_MODE_NORMAL           0   /* 透传模式 */
#define LANYA_MODE_CONFIG           1   /* AT 指令配置模式 */

/* ========== 用户接口函数 ========== */

/**
 * @brief  初始化 E104 蓝牙驱动
 *         须在 SYSCFG_DL_init() 之后调用
 */
void Lanya_Init(void);

/**
 * @brief  通过蓝牙发送数据
 * @param data  数据缓冲区
 * @param len   数据长度
 */
void Lanya_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief  通过蓝牙发送字符串
 * @param str  以 '\0' 结尾的字符串
 */
void Lanya_SendString(const char *str);

/**
 * @brief  从环形缓冲区读取已接收的数据
 * @param buf    存放数据的缓冲区
 * @param maxLen 缓冲区最大长度
 * @return 实际读取的字节数
 */
uint16_t Lanya_GetReceivedData(uint8_t *buf, uint16_t maxLen);

/**
 * @brief  检查是否有新数据可读
 * @return true=有数据, false=无数据
 */
bool Lanya_IsDataAvailable(void);

/**
 * @brief  设置 E104 工作模式
 * @param mode  LANYA_MODE_NORMAL 或 LANYA_MODE_CONFIG
 */
void Lanya_SetMode(uint8_t mode);

/**
 * @brief  UART 接收回调（从你的 UART_IRQHandler 中调用）
 * @param rx_byte  收到的字节
 *
 * 用法：在你的 UART_IRQHandler 中调用此函数：
 *
 *   void LANYA_UART_IRQHandler(void)
 *   {
 *       switch (DL_UART_Main_getPendingInterrupt(LANYA_UART_INST))
 *       {
 *           case DL_UART_MAIN_IIDX_RX: {
 *               uint8_t byte;
 *               while (DL_UART_Main_receiveDataCheck(LANYA_UART_INST, &byte))
 *               {
 *                   Lanya_RxCallback(byte);
 *               }
 *               break;
 *           }
 *           default: break;
 *       }
 *   }
 */
void Lanya_RxCallback(uint8_t rx_byte);

#endif /* __COMMUNICATE_LANYA_MSP_H */
