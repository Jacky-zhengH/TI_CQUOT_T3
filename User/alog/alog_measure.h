#ifndef ALOG_MEASURE_H
#define ALOG_MEASURE_H

#include <stdint.h>

/* 置1明确表示当前物理量仅为跨项目参考模拟值，不是本机标定结果。 */
#define ALOG_REFERENCE_SIMULATION_ONLY (1)

/** @brief 普通uint16_t算术平均；空数组返回0。 */
uint16_t alog_average_u16(const uint16_t *data, uint16_t count);
/** @brief 使用uint64_t累加，避免周期Tick求和溢出。 */
uint32_t alog_average_u32(const uint32_t *data, uint16_t count);
/** @brief 按有符号16.16原始值平均，返回位模式仍为uint32_t。 */
uint32_t alog_average_tdc(const uint32_t *data, uint16_t count);

/** @brief 使用参考模拟系数把GP22 RES0换算为厘米。 */
float alog_length_reference_cm(uint32_t raw);
/** @brief 按62 ohm分压和电缆寄生参考系数计算终端电阻。 */
float alog_resistance_reference_ohm(uint16_t adc,
                                    float cable_length_cm);
/** @brief 按NE555周期和电缆寄生参考系数计算终端电容。 */
float alog_capacitance_reference_pf(uint32_t period_ticks,
                                    float cable_length_cm);

#endif /* ALOG_MEASURE_H */
