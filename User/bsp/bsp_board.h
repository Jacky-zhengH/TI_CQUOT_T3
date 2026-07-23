#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/*
 * 极简公共时间基准
 * ----------------
 * HAL_GetTick()由SysTick每1 ms累加一次。这里仅统一名称，不维护额外计数器，
 * 也不需要初始化。上层保存“开始时刻”，随后用bsp_timeout()判断是否到期。
 */

/**
 * @brief 返回HAL提供的当前毫秒Tick。
 * @return 从系统启动后累计的毫秒计数；约49.7天后会自然回绕。
 */
static inline uint32_t bsp_now_ms(void)
{
    return HAL_GetTick();
}

/**
 * @brief 判断从start_ms开始是否已经达到timeout_ms。
 * @param start_ms 由bsp_now_ms()记录的开始时刻。
 * @param timeout_ms 等待上限，单位为毫秒。
 * @return 已到期返回true，否则返回false。
 * @note 必须保留uint32_t无符号减法，才能在Tick回绕时继续得到正确间隔。
 */
static inline bool bsp_timeout(uint32_t start_ms, uint32_t timeout_ms)
{
    return (uint32_t)(bsp_now_ms() - start_ms) >= timeout_ms;
}

#endif /* BSP_BOARD_H */
