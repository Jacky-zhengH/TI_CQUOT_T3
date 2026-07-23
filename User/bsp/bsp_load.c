#include "bsp_load.h"

#include <stddef.h>

#include "adc.h"
#include "tim.h"

/* ADC前4次只用于节点建立；正式数组与TIM5内部数组均固定上限100。 */
#define BSP_LOAD_ADC_DISCARD_COUNT (4U)
#define BSP_LOAD_ADC_MAX_COUNT     (100U)
#define BSP_LOAD_ADC_TIMEOUT_MS    (2U)
#define BSP_LOAD_PERIOD_MAX_COUNT  (100U)

/* 这些变量由TIM5 ISR写、主循环读，因此必须保持volatile。 */
static volatile uint32_t g_captured_periods[BSP_LOAD_PERIOD_MAX_COUNT];
static volatile uint32_t g_previous_capture;
static volatile uint16_t g_period_count;
static volatile uint16_t g_period_target;
static volatile bool g_first_capture_seen;
static volatile bool g_capture_active;
static volatile bool g_capture_overrun;

static bool adc_read_once(uint16_t *sample)
{
    HAL_StatusTypeDef result;

    /* 每个样本严格执行Start -> Poll -> Get -> Stop，任何错误都向上返回。 */
    result = HAL_ADC_Start(&hadc1);
    if (result != HAL_OK)
    {
        return false;
    }

    result = HAL_ADC_PollForConversion(&hadc1,
                                       BSP_LOAD_ADC_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);
        return false;
    }

    *sample = (uint16_t)HAL_ADC_GetValue(&hadc1);
    return HAL_ADC_Stop(&hadc1) == HAL_OK;
}

static void capture_reset(void)
{
    g_previous_capture = 0U;
    g_period_count = 0U;
    g_period_target = 0U;
    g_first_capture_seen = false;
    g_capture_active = false;
    g_capture_overrun = false;
}

void bsp_load_init(void)
{
    if (hadc1.Instance == ADC1)
    {
        (void)HAL_ADC_Stop(&hadc1);
    }
    if (htim5.Instance == TIM5)
    {
        (void)HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2);
    }
    capture_reset();
}

bool bsp_load_read_adc(uint16_t *samples, uint16_t count)
{
    uint16_t discarded_sample;
    uint16_t index;

    if ((samples == NULL) || (count == 0U) ||
        (count > BSP_LOAD_ADC_MAX_COUNT) ||
        (hadc1.Instance != ADC1))
    {
        return false;
    }

    for (index = 0U; index < BSP_LOAD_ADC_DISCARD_COUNT; index++)
    {
        /* 切换继电器后先让分压节点和ADC内部采样电容稳定。 */
        if (!adc_read_once(&discarded_sample))
        {
            return false;
        }
    }

    for (index = 0U; index < count; index++)
    {
        if (!adc_read_once(&samples[index]))
        {
            return false;
        }
    }
    return true;
}

bool bsp_load_capture_periods(uint32_t *periods,
                              uint16_t count,
                              uint32_t timeout_ms)
{
    uint32_t start_ms;
    uint16_t index;
    bool success;

    if ((periods == NULL) || (count == 0U) ||
        (count > BSP_LOAD_PERIOD_MAX_COUNT) ||
        (timeout_ms == 0U) || (htim5.Instance != TIM5))
    {
        return false;
    }

    capture_reset();
    g_period_target = count;
    /* 第一条新边沿只建立起点，所以每轮都清计数器、CC2及过捕获标志。 */
    __HAL_TIM_SET_COUNTER(&htim5, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC2);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC2OF);
    g_capture_active = true;

    if (HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2) != HAL_OK)
    {
        g_capture_active = false;
        return false;
    }

    start_ms = HAL_GetTick();
    while ((g_period_count < count) && (!g_capture_overrun) &&
           ((HAL_GetTick() - start_ms) < timeout_ms))
    {
        /* 主循环只做有界等待；周期数组由TIM5捕获回调填充。 */
        if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC2OF) != RESET)
        {
            g_capture_overrun = true;
        }
    }

    g_capture_active = false;
    success = (g_period_count >= count) && (!g_capture_overrun);
    /* 成功、超时或CC2OF都必须关闭捕获中断，保证下一条命令可重试。 */
    if (HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2) != HAL_OK)
    {
        success = false;
    }
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC2OF);

    if (!success)
    {
        return false;
    }

    for (index = 0U; index < count; index++)
    {
        periods[index] = g_captured_periods[index];
    }
    return true;
}

void bsp_load_on_tim_capture_irq(TIM_HandleTypeDef *htim)
{
    uint32_t capture;
    uint16_t index;

    if ((htim == NULL) || (htim->Instance != TIM5) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2) ||
        (!g_capture_active))
    {
        return;
    }

    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2OF) != RESET)
    {
        /* CCR2被新边沿覆盖后整组周期不可信，立即通知主循环失败。 */
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2OF);
        g_capture_overrun = true;
        g_capture_active = false;
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
        return;
    }

    capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    if (!g_first_capture_seen)
    {
        /* 第一条上升沿没有前驱，仅保存为下一周期的起点。 */
        g_previous_capture = capture;
        g_first_capture_seen = true;
        return;
    }

    index = g_period_count;
    if ((index >= g_period_target) ||
        (index >= BSP_LOAD_PERIOD_MAX_COUNT))
    {
        g_capture_active = false;
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
        return;
    }

    /* uint32_t无符号减法天然兼容TIM5计数器回绕。 */
    g_captured_periods[index] = capture - g_previous_capture;
    g_previous_capture = capture;
    index++;
    g_period_count = index;
    if (index >= g_period_target)
    {
        g_capture_active = false;
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
    }
}
