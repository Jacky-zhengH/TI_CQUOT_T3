#ifndef BSP_LOAD_H
#define BSP_LOAD_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/* APB1定时器84 MHz / (PSC 41 + 1)，每个捕获Tick为0.5 us。 */
#define TIM5_CAPTURE_TICK_HZ (2000000.0f)

/** @brief 停止残留ADC/TIM5操作并清空捕获现场。 */
void bsp_load_init(void);

/** @brief 丢弃前4次转换后，读取指定数量的ADC1_IN10原始码。 */
bool bsp_load_read_adc(uint16_t *samples,
                       uint16_t count);

/** @brief 有限等待TIM5_CH2相邻上升沿周期，失败和成功都会停止捕获。 */
bool bsp_load_capture_periods(uint32_t *periods,
                              uint16_t count,
                              uint32_t timeout_ms);

/** @brief TIM5捕获回调入口；ISR中只保存Tick差和错误标志。 */
void bsp_load_on_tim_capture_irq(TIM_HandleTypeDef *htim);

#endif /* BSP_LOAD_H */
