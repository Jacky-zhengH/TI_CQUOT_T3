#ifndef BSP_LENGTH_H
#define BSP_LENGTH_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 硬复位GP22、写入固定配置并通过Reg1读回验证SPI通信。 */
bool bsp_length_init(void);

/** @brief 触发一次TIM1单脉冲，有限等待INTN并返回STAT和RES0原始值。 */
bool bsp_length_measure(uint32_t *raw,
                        uint16_t *status,
                        uint32_t timeout_ms);

/** @brief PE0 EXTI转发入口；中断上下文中只设置volatile标志。 */
void bsp_length_on_gp22_irq(uint16_t gpio_pin);

#endif /* BSP_LENGTH_H */
