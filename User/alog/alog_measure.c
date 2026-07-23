#include "alog_measure.h"

#include <stddef.h>

bool alog_average_i32(const int32_t *samples,
                      uint16_t count,
                      int32_t *average)
{
    int64_t sum = 0;
    uint16_t index;

    if ((samples == NULL) || (average == NULL) || (count == 0U))
    {
        return false;
    }
    for (index = 0U; index < count; index++)
    {
        sum += samples[index];
    }
    *average = (int32_t)(sum / (int64_t)count);
    return true;
}

bool alog_average_u16(const uint16_t *samples,
                      uint16_t count,
                      uint16_t *average)
{
    uint32_t sum = 0U;
    uint16_t index;

    if ((samples == NULL) || (average == NULL) || (count == 0U))
    {
        return false;
    }
    for (index = 0U; index < count; index++)
    {
        sum += samples[index];
    }
    *average = (uint16_t)(sum / (uint32_t)count);
    return true;
}

bool alog_average_u32(const uint32_t *samples,
                      uint16_t count,
                      uint32_t *average)
{
    uint64_t sum = 0U;
    uint16_t index;

    if ((samples == NULL) || (average == NULL) || (count == 0U))
    {
        return false;
    }
    for (index = 0U; index < count; index++)
    {
        sum += samples[index];
    }
    *average = (uint32_t)(sum / (uint64_t)count);
    return true;
}
