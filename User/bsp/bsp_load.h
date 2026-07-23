#ifndef BSP_LOAD_H
#define BSP_LOAD_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * 负载原始采集模块
 * ----------------
 * 电阻通路返回ADC原始码，电容通路返回TIM5相邻边沿的周期Tick。
 * 本模块不计算欧姆、皮法或负载类型，这些统计和标定工作由ALOG层完成。
 */

typedef enum
{
    BSP_LOAD_RESULT_OK = 0,       /* 指定数量的原始样本已经写入调用者缓冲区。 */
    BSP_LOAD_RESULT_INVALID_PARAM,/* 空指针、零数量、数量过大或零超时。 */
    BSP_LOAD_RESULT_NOT_READY,    /* 外设句柄错误，或对应继电器通路没有接通。 */
    BSP_LOAD_RESULT_ADC_ERROR,    /* ADC启动、转换或停止过程中发生HAL错误。 */
    BSP_LOAD_RESULT_TIMER_ERROR,  /* TIM5输入捕获启动或停止失败。 */
    BSP_LOAD_RESULT_TIMEOUT,      /* ADC单次转换或整组周期捕获超过时间上限。 */
    BSP_LOAD_RESULT_OVERRUN       /* CC2OF置位，说明新边沿覆盖了未处理捕获值。 */
} bsp_load_result_t;

/** @brief 清理ADC和TIM5采集现场。 */
void bsp_load_init(void);

/**
 * @brief 丢弃4次转换后，同步读取指定数量的ADC原始样本。
 * @param samples 调用者提供的uint16_t数组。
 * @param count 需要保存的样本数，范围1~64。
 * @return 采集结果；函数返回OK时数组内容才完整有效。
 */
bsp_load_result_t bsp_load_read_adc(uint16_t *samples, uint16_t count);

/**
 * @brief 在有限超时内捕获指定数量的NE555周期Tick。
 * @param periods 调用者提供的uint32_t周期数组。
 * @param count 需要保存的周期数，范围1~16。
 * @param timeout_ms 整组捕获的最大等待时间，单位为毫秒。
 * @return 捕获结果；函数返回OK时数组内容才完整有效。
 */
bsp_load_result_t bsp_load_capture_periods(uint32_t *periods,
                                            uint16_t count,
                                            uint32_t timeout_ms);

/**
 * @brief 返回TIM5输入捕获计数器的实际Tick频率。
 * @return 由APB1时钟和TIM5预分频器计算得到的频率，配置异常时返回0。
 */
uint32_t bsp_load_get_capture_tick_hz(void);

/**
 * @brief 处理HAL转发的TIM5通道2输入捕获中断。
 * @note ISR中只保存Tick和标志，不执行阻塞、日志、换算或浮点计算。
 */
void bsp_load_on_tim_capture_irq(TIM_HandleTypeDef *htim);

#endif /* BSP_LOAD_H */
