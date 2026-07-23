#include "alog_measure.h"

#include <stddef.h>

/*
 * 仓库中没有电阻分压原理图，当前仅能集中选择一种接法：
 * 1：参考电阻接3.3 V，未知负载接地，ADC读取两者中点（当前默认）。
 * 0：未知负载接3.3 V，参考电阻接地，ADC读取两者中点。
 * 上板前必须根据原理图或用已知电阻实测确认，APP不感知该硬件方向。
 */
#define ALOG_RESISTANCE_REFERENCE_TO_VREF (1U)

/* STM32F407的12位ADC满量程码为4095，预留4码防止靠近电源轨时除数失稳。 */
#define ALOG_ADC_FULL_SCALE_CODE           (4095U)
#define ALOG_ADC_RAIL_GUARD_CODE           (4U)

#define ALOG_LENGTH_MIN_M                  (0.1f)
#define ALOG_LENGTH_MAX_M                  (30.0f)
#define ALOG_CAPACITANCE_MIN_PF            (50.0f)
#define ALOG_CAPACITANCE_MAX_PF            (400.0f)

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

bool alog_stats_u16(const uint16_t *samples,
                    uint16_t count,
                    uint16_t *minimum,
                    uint16_t *maximum,
                    uint16_t *average)
{
    uint32_t sum = 0U;
    uint16_t min_value;
    uint16_t max_value;
    uint16_t index;

    if ((samples == NULL) || (count == 0U) ||
        (minimum == NULL) || (maximum == NULL) || (average == NULL))
    {
        return false;
    }

    min_value = samples[0];
    max_value = samples[0];
    for (index = 0U; index < count; index++)
    {
        if (samples[index] < min_value)
        {
            min_value = samples[index];
        }
        if (samples[index] > max_value)
        {
            max_value = samples[index];
        }
        sum += samples[index];
    }

    *minimum = min_value;
    *maximum = max_value;
    *average = (uint16_t)(sum / (uint32_t)count);
    return true;
}

bool alog_stats_u32(const uint32_t *samples,
                    uint16_t count,
                    uint32_t *minimum,
                    uint32_t *maximum,
                    uint32_t *average)
{
    uint64_t sum = 0U;
    uint32_t min_value;
    uint32_t max_value;
    uint16_t index;

    if ((samples == NULL) || (count == 0U) ||
        (minimum == NULL) || (maximum == NULL) || (average == NULL))
    {
        return false;
    }

    min_value = samples[0];
    max_value = samples[0];
    for (index = 0U; index < count; index++)
    {
        if (samples[index] < min_value)
        {
            min_value = samples[index];
        }
        if (samples[index] > max_value)
        {
            max_value = samples[index];
        }
        sum += samples[index];
    }

    *minimum = min_value;
    *maximum = max_value;
    *average = (uint32_t)(sum / (uint64_t)count);
    return true;
}

bool alog_length_from_raw(int32_t raw,
                          const bsp_calibration_data_t *cal,
                          float *length_m)
{
    float calculated_length_m;

    if ((cal == NULL) || (length_m == NULL) ||
        ((cal->valid_mask & BSP_CAL_VALID_LENGTH) == 0U))
    {
        return false;
    }

    calculated_length_m =
        ((cal->length_gain_cm_per_raw * (float)raw) +
         cal->length_offset_cm) /
        100.0f;
    /* 合并范围判断也会拒绝NaN，避免损坏标定记录产生虚假有效值。 */
    if (!((calculated_length_m >= ALOG_LENGTH_MIN_M) &&
          (calculated_length_m <= ALOG_LENGTH_MAX_M)))
    {
        return false;
    }

    *length_m = calculated_length_m;
    return true;
}

bool alog_resistance_from_adc(uint16_t adc_average,
                              float length_m,
                              const bsp_calibration_data_t *cal,
                              float *resistance_ohm)
{
    float ratio;
    float total_resistance_ohm;
    float calculated_resistance_ohm;

    if ((cal == NULL) || (resistance_ohm == NULL) ||
        ((cal->valid_mask & BSP_CAL_VALID_RESISTANCE) == 0U) ||
        (!(cal->resistance_reference_ohm > 0.0f)) ||
        (!(length_m >= 0.0f)) ||
        (adc_average <= ALOG_ADC_RAIL_GUARD_CODE) ||
        (adc_average >=
         (ALOG_ADC_FULL_SCALE_CODE - ALOG_ADC_RAIL_GUARD_CODE)))
    {
        return false;
    }

    ratio = (float)adc_average / (float)ALOG_ADC_FULL_SCALE_CODE;
#if ALOG_RESISTANCE_REFERENCE_TO_VREF
    total_resistance_ohm =
        cal->resistance_reference_ohm * ratio / (1.0f - ratio);
#else
    total_resistance_ohm =
        cal->resistance_reference_ohm * (1.0f - ratio) / ratio;
#endif

    calculated_resistance_ohm =
        (cal->resistance_gain * total_resistance_ohm) +
        cal->resistance_offset_ohm -
        (cal->cable_resistance_ohm_per_m * length_m);
    if (!(calculated_resistance_ohm >= 0.0f))
    {
        return false;
    }

    *resistance_ohm = calculated_resistance_ohm;
    return true;
}

bool alog_capacitance_from_period(uint32_t open_period_ticks,
                                  uint32_t load_period_ticks,
                                  const bsp_calibration_data_t *cal,
                                  float *capacitance_pf)
{
    uint32_t delta_ticks;
    float calculated_capacitance_pf;

    if ((cal == NULL) || (capacitance_pf == NULL) ||
        ((cal->valid_mask & BSP_CAL_VALID_CAPACITANCE) == 0U) ||
        (load_period_ticks <= open_period_ticks))
    {
        return false;
    }

    delta_ticks = load_period_ticks - open_period_ticks;
    calculated_capacitance_pf =
        (cal->capacitance_gain_pf_per_tick * (float)delta_ticks) +
        cal->capacitance_offset_pf;
    if (!((calculated_capacitance_pf >= ALOG_CAPACITANCE_MIN_PF) &&
          (calculated_capacitance_pf <= ALOG_CAPACITANCE_MAX_PF)))
    {
        return false;
    }

    *capacitance_pf = calculated_capacitance_pf;
    return true;
}
