#ifndef ALOG_MEASURE_H
#define ALOG_MEASURE_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_flash.h"

/* 原始样本统计；物理量换算将在获得实测标定参数后继续放在本层。 */
bool alog_average_i32(const int32_t *samples,
                      uint16_t count,
                      int32_t *average);
bool alog_average_u16(const uint16_t *samples,
                      uint16_t count,
                      uint16_t *average);
bool alog_average_u32(const uint32_t *samples,
                      uint16_t count,
                      uint32_t *average);

/** @brief 统计uint16_t数组的最小值、最大值和平均值。 */
bool alog_stats_u16(const uint16_t *samples,
                    uint16_t count,
                    uint16_t *minimum,
                    uint16_t *maximum,
                    uint16_t *average);

/** @brief 统计uint32_t数组的最小值、最大值和平均值。 */
bool alog_stats_u32(const uint32_t *samples,
                    uint16_t count,
                    uint32_t *minimum,
                    uint32_t *maximum,
                    uint32_t *average);

/** @brief 使用Flash线性标定将GP22原始值换算为电缆长度。 */
bool alog_length_from_raw(int32_t raw,
                          const bsp_calibration_data_t *cal,
                          float *length_m);

/** @brief 使用分压模型和Flash标定计算终端负载电阻。 */
bool alog_resistance_from_adc(uint16_t adc_average,
                              float length_m,
                              const bsp_calibration_data_t *cal,
                              float *resistance_ohm);

/** @brief 使用开路周期差和Flash线性标定计算终端电容。 */
bool alog_capacitance_from_period(uint32_t open_period_ticks,
                                  uint32_t load_period_ticks,
                                  const bsp_calibration_data_t *cal,
                                  float *capacitance_pf);

#endif /* ALOG_MEASURE_H */
