#ifndef ALOG_MEASURE_H
#define ALOG_MEASURE_H

#include <stdbool.h>
#include <stdint.h>

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

#endif /* ALOG_MEASURE_H */
