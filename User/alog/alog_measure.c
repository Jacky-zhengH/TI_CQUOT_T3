#include "alog_measure.h"

#include <stddef.h>

#include "bsp_load.h"

/*
 * 下列系数借自成功项目，只用于检查“原始量 -> 参考物理量”计算链。
 * 本机完成10/15/20 m、10/20/30 ohm、100/200/300 pF标定前不得作为精度指标。
 */
#define ALOG_GP22_FE9                    (0.002f)
#define ALOG_PROPAGATION_FACTOR          (2.01546f)
#define ALOG_LENGTH_BASE_CM              (16.7f)

#define ALOG_ADC_REFERENCE_VOLTAGE       (3.3f)
#define ALOG_ADC_CODE_DIVISOR            (4096.0f)
#define ALOG_DIVIDER_REFERENCE_OHM       (62.0f)
#define ALOG_VOLTAGE_GAIN                (1.0072f)
#define ALOG_VOLTAGE_OFFSET              (-0.0007f)
#define ALOG_RESISTANCE_GAIN             (0.8615f)
#define ALOG_RESISTANCE_OFFSET_OHM       (-0.1298f)
#define ALOG_CABLE_R_OHM_PER_CM          (0.0014f)
#define ALOG_CABLE_R_OFFSET_OHM          (0.0328f)

#define ALOG_NE555_R1_MOHM               (0.01f)
#define ALOG_NE555_R2_MOHM               (1.0f)
#define ALOG_NE555_FACTOR                (1.442695f)
#define ALOG_FREQUENCY_GAIN              (1.0048f)
#define ALOG_FREQUENCY_OFFSET_HZ         (-0.0501f)
#define ALOG_CAPACITANCE_GAIN            (1.003054914f)
#define ALOG_CAPACITANCE_OFFSET_PF       (36.6104f)
#define ALOG_CABLE_C_PF_PER_CM           (0.9527f)
#define ALOG_CABLE_C_OFFSET_PF           (1.4487f)

#define ALOG_INVALID_RESULT              (-1.0f)

uint16_t alog_average_u16(const uint16_t *data, uint16_t count)
{
    uint32_t sum = 0U;
    uint16_t index;

    if ((data == NULL) || (count == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < count; index++)
    {
        sum += data[index];
    }
    return (uint16_t)(sum / count);
}

uint32_t alog_average_u32(const uint32_t *data, uint16_t count)
{
    uint64_t sum = 0U;
    uint16_t index;

    if ((data == NULL) || (count == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < count; index++)
    {
        sum += data[index];
    }
    return (uint32_t)(sum / count);
}

uint32_t alog_average_tdc(const uint32_t *data, uint16_t count)
{
    int64_t sum = 0;
    int32_t average;
    uint16_t index;

    if ((data == NULL) || (count == 0U))
    {
        return 0U;
    }

    for (index = 0U; index < count; index++)
    {
        /* GP22 RES0是有符号16.16，求和前必须按int32_t恢复符号。 */
        sum += (int32_t)data[index];
    }
    average = (int32_t)(sum / (int64_t)count);
    return (uint32_t)average;
}

float alog_length_reference_cm(uint32_t raw)
{
    float signed_fixed;
    float time_ns;

    /* 高16位为有符号整数、低16位为小数，不能直接把raw转成float。 */
    signed_fixed = (float)(int16_t)(raw >> 16U);
    signed_fixed += (float)(raw & 0xFFFFU) / 65536.0f;
    time_ns = signed_fixed / ALOG_GP22_FE9;
    return (time_ns * ALOG_PROPAGATION_FACTOR * 0.5f * 10.0f) -
           ALOG_LENGTH_BASE_CM;
}

float alog_resistance_reference_ohm(uint16_t adc,
                                    float cable_length_cm)
{
    float voltage_raw;
    float voltage_sim;
    float denominator;
    float resistance_calculated;
    float resistance_sim;
    float cable_resistance;
    float load_resistance;

    voltage_raw = (float)adc * ALOG_ADC_REFERENCE_VOLTAGE /
                  ALOG_ADC_CODE_DIVISOR;
    voltage_sim = (voltage_raw * ALOG_VOLTAGE_GAIN) +
                  ALOG_VOLTAGE_OFFSET;
    denominator = ALOG_ADC_REFERENCE_VOLTAGE - voltage_sim;
    if ((voltage_sim <= 0.0f) || (denominator <= 0.0f))
    {
        return ALOG_INVALID_RESULT;
    }

    /*
     * 实际拓扑必须是3.3 V -> 62 ohm -> ADC节点 -> 负载 -> GND，
     * 因而Rload = 62 * Vadc / (3.3 - Vadc)。
     */
    resistance_calculated = ALOG_DIVIDER_REFERENCE_OHM *
                            voltage_sim / denominator;
    resistance_sim = (ALOG_RESISTANCE_GAIN * resistance_calculated) +
                     ALOG_RESISTANCE_OFFSET_OHM;
    cable_resistance = (ALOG_CABLE_R_OHM_PER_CM * cable_length_cm) +
                       ALOG_CABLE_R_OFFSET_OHM;
    load_resistance = resistance_sim - cable_resistance;
    return (load_resistance >= 0.0f) ?
           load_resistance : ALOG_INVALID_RESULT;
}

float alog_capacitance_reference_pf(uint32_t period_ticks,
                                    float cable_length_cm)
{
    float frequency_raw;
    float frequency_sim;
    float resistance_sum;
    float total_calculated_pf;
    float total_sim_pf;
    float cable_capacitance_pf;
    float load_capacitance_pf;

    if (period_ticks == 0U)
    {
        return ALOG_INVALID_RESULT;
    }

    /* TIM5返回周期Tick；先换成频率，再代入NE555无稳态公式。 */
    frequency_raw = TIM5_CAPTURE_TICK_HZ / (float)period_ticks;
    frequency_sim = (frequency_raw * ALOG_FREQUENCY_GAIN) +
                    ALOG_FREQUENCY_OFFSET_HZ;
    resistance_sum = ALOG_NE555_R1_MOHM +
                     (2.0f * ALOG_NE555_R2_MOHM);
    if ((frequency_sim <= 0.0f) || (resistance_sum <= 0.0f))
    {
        return ALOG_INVALID_RESULT;
    }

    total_calculated_pf = 1000000.0f * ALOG_NE555_FACTOR /
                          resistance_sum / frequency_sim;
    total_sim_pf = ALOG_CAPACITANCE_GAIN *
                   (total_calculated_pf -
                    ALOG_CAPACITANCE_OFFSET_PF);
    /* 总电容减去按最近长度估计的电缆寄生，才得到终端负载参考值。 */
    cable_capacitance_pf =
        (ALOG_CABLE_C_PF_PER_CM * cable_length_cm) +
        ALOG_CABLE_C_OFFSET_PF;
    load_capacitance_pf = total_sim_pf - cable_capacitance_pf;
    return (load_capacitance_pf >= 0.0f) ?
           load_capacitance_pf : ALOG_INVALID_RESULT;
}
