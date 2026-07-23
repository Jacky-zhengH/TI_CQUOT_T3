#include "bsp_load.h"

#include <stdbool.h>
#include <stddef.h>
#include "bsp_board.h"
#include "adc.h"
#include "main.h"
#include "tim.h"

/*
 * ADC切换到电阻分压节点后，内部采样电容需要短暂建立，因此先丢弃4次结果。
 * 64是同步接口允许的最大数组长度，防止错误参数造成过长阻塞；APP当前用32点。
 * 单次转换等待2 ms，远大于当前ADC实际转换时间，同时能在外设异常时快速退出。
 */
#define BSP_LOAD_ADC_DISCARD_COUNT (4U)
#define BSP_LOAD_ADC_MAX_SAMPLE_COUNT (64U)
#define BSP_LOAD_ADC_TIMEOUT_MS (2U)

/* 当前竞赛流程采集16个555周期；固定上限同时也是ISR静态数组的边界。 */
#define BSP_LOAD_PERIOD_MAX_COUNT (16U)

/*
 * 以下变量由TIM5中断写、主循环读，因此使用volatile。
 * s_periods只保存相邻两次捕获值之差，不保存最终频率或电容值。
 */
static volatile uint32_t s_periods[BSP_LOAD_PERIOD_MAX_COUNT];
static volatile uint32_t s_last_capture_tick;
static volatile uint16_t s_period_count;
static volatile uint16_t s_period_target;
static volatile bool s_has_first_capture;
static volatile bool s_capture_active;
static volatile bool s_capture_overrun;

static bool bsp_load_path_selected(uint16_t active_pin)
{
    GPIO_PinState length_level;
    GPIO_PinState resistance_level;
    GPIO_PinState capacitance_level;

    /*
     * 不只检查目标继电器为ON，还同时确认另外两路为OFF。
     * 这样APP若忘记选择通路，或互锁输出异常，BSP会拒绝采集。
     */
    length_level = HAL_GPIO_ReadPin(PATH_LENGTH_EN_GPIO_Port,
                                    PATH_LENGTH_EN_Pin);
    resistance_level = HAL_GPIO_ReadPin(PATH_RESISTANCE_EN_GPIO_Port,
                                        PATH_RESISTANCE_EN_Pin);
    capacitance_level = HAL_GPIO_ReadPin(PATH_CAPACITANCE_EN_GPIO_Port,
                                         PATH_CAPACITANCE_EN_Pin);

    if (active_pin == PATH_RESISTANCE_EN_Pin)
    {
        return (length_level == GPIO_PIN_RESET) &&
               (resistance_level == GPIO_PIN_SET) &&
               (capacitance_level == GPIO_PIN_RESET);
    }
    if (active_pin == PATH_CAPACITANCE_EN_Pin)
    {
        return (length_level == GPIO_PIN_RESET) &&
               (resistance_level == GPIO_PIN_RESET) &&
               (capacitance_level == GPIO_PIN_SET);
    }
    return false;
}

static bsp_load_result_t bsp_load_adc_once(uint16_t *sample)
{
    HAL_StatusTypeDef hal_status;

    /* 一次完整转换严格遵循Start -> Poll -> Get -> Stop。 */
    hal_status = HAL_ADC_Start(&hadc1);
    if (hal_status != HAL_OK)
    {
        return BSP_LOAD_RESULT_ADC_ERROR;
    }

    hal_status = HAL_ADC_PollForConversion(&hadc1,
                                           BSP_LOAD_ADC_TIMEOUT_MS);
    if (hal_status != HAL_OK)
    {
        /* 即使轮询失败也尝试停止ADC，避免故障状态影响下一次测量。 */
        (void)HAL_ADC_Stop(&hadc1);
        return (hal_status == HAL_TIMEOUT) ? BSP_LOAD_RESULT_TIMEOUT : BSP_LOAD_RESULT_ADC_ERROR;
    }

    *sample = (uint16_t)HAL_ADC_GetValue(&hadc1);
    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
    {
        return BSP_LOAD_RESULT_ADC_ERROR;
    }
    return BSP_LOAD_RESULT_OK;
}

static void bsp_load_reset_capture(void)
{
    /* 第一条边沿只建立起点，因此每轮捕获前必须清除上一轮基准和计数。 */
    s_last_capture_tick = 0U;
    s_period_count = 0U;
    s_period_target = 0U;
    s_has_first_capture = false;
    s_capture_active = false;
    s_capture_overrun = false;
}

void bsp_load_init(void)
{
    /* 初始化不重新配置CubeMX外设，只停止可能残留的活动并清软件现场。 */
    if (hadc1.Instance == ADC1)
    {
        (void)HAL_ADC_Stop(&hadc1);
    }
    if (htim5.Instance == TIM5)
    {
        (void)HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2);
    }
    bsp_load_reset_capture();
}

bsp_load_result_t bsp_load_read_adc(uint16_t *samples, uint16_t count)
{
    bsp_load_result_t result;
    uint16_t sample;
    uint16_t index;

    if ((samples == NULL) || (count == 0U) ||
        (count > BSP_LOAD_ADC_MAX_SAMPLE_COUNT))
    {
        return BSP_LOAD_RESULT_INVALID_PARAM;
    }
    if ((hadc1.Instance != ADC1) ||
        (!bsp_load_path_selected(PATH_RESISTANCE_EN_Pin)))
    {
        return BSP_LOAD_RESULT_NOT_READY;
    }

    /* 前4次只用于让模拟节点和ADC采样电容稳定，不写入调用者数组。 */
    for (index = 0U; index < BSP_LOAD_ADC_DISCARD_COUNT; index++)
    {
        result = bsp_load_adc_once(&sample);
        if (result != BSP_LOAD_RESULT_OK)
        {
            return result;
        }
    }

    /* 稳定后才连续保存真正参与ALOG统计的原始码值。 */
    for (index = 0U; index < count; index++)
    {
        result = bsp_load_adc_once(&samples[index]);
        if (result != BSP_LOAD_RESULT_OK)
        {
            return result;
        }
    }
    return BSP_LOAD_RESULT_OK;
}

bsp_load_result_t bsp_load_capture_periods(uint32_t *periods,
                                           uint16_t count,
                                           uint32_t timeout_ms)
{
    uint32_t start_ms;
    uint16_t index;

    if ((periods == NULL) || (count == 0U) ||
        (count > BSP_LOAD_PERIOD_MAX_COUNT) || (timeout_ms == 0U))
    {
        return BSP_LOAD_RESULT_INVALID_PARAM;
    }
    if ((htim5.Instance != TIM5) ||
        (!bsp_load_path_selected(PATH_CAPACITANCE_EN_Pin)))
    {
        return BSP_LOAD_RESULT_NOT_READY;
    }

    /*
     * 发布capture_active之前清计数器和旧标志，保证第一个新边沿不会与
     * 上一轮数据混在一起。一个有效周期需要“起点边沿+终点边沿”。
     */
    bsp_load_reset_capture();
    s_period_target = count;
    s_capture_active = true;
    __HAL_TIM_SET_COUNTER(&htim5, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC2);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC2OF);

    if (HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2) != HAL_OK)
    {
        s_capture_active = false;
        return BSP_LOAD_RESULT_TIMER_ERROR;
    }

    /* 主循环有界等待；真正的捕获和数组写入由TIM5 ISR完成。 */
    start_ms = bsp_now_ms();
    while ((s_period_count < count) &&
           (!s_capture_overrun) &&
           (!bsp_timeout(start_ms, timeout_ms)))
    {
        /* 周期由TIM5 ISR写入；此处只做有明确上限的等待。 */
    }

    s_capture_active = false;
    if (HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2) != HAL_OK)
    {
        return BSP_LOAD_RESULT_TIMER_ERROR;
    }
    if (s_capture_overrun)
    {
        return BSP_LOAD_RESULT_OVERRUN;
    }
    if (s_period_count < count)
    {
        return BSP_LOAD_RESULT_TIMEOUT;
    }

    /* 停止中断后再复制，避免复制过程中数组仍被ISR修改。 */
    for (index = 0U; index < count; index++)
    {
        periods[index] = s_periods[index];
    }
    return BSP_LOAD_RESULT_OK;
}

uint32_t bsp_load_get_capture_tick_hz(void)
{
    uint32_t timer_clock_hz;

    if ((htim5.Instance != TIM5) ||
        (htim5.Init.Prescaler == UINT32_MAX))
    {
        return 0U;
    }

    /*
     * STM32F4的APB1分频不为1时，定时器输入时钟为PCLK1的2倍。
     * 计数Tick频率还需要除以(PSC + 1)，避免APP固定假设为2 MHz。
     */
    timer_clock_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_clock_hz *= 2U;
    }

    return timer_clock_hz / (htim5.Init.Prescaler + 1U);
}

void bsp_load_on_tim_capture_irq(TIM_HandleTypeDef *htim)
{
    uint32_t captured_tick;
    uint16_t period_index;

    if ((htim == NULL) || (htim->Instance != TIM5) ||
        (!s_capture_active))
    {
        return;
    }

    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2OF) != RESET)
    {
        /*
         * CC2OF表示CCR2还未处理就到达了下一边沿，本次周期序列不再可信。
         * ISR立即关CC2中断，主循环随后返回OVERRUN。
         */
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2OF);
        s_capture_overrun = true;
        s_capture_active = false;
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
        return;
    }
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2)
    {
        return;
    }

    captured_tick = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    if (!s_has_first_capture)
    {
        /* 第一条边沿没有前一个时间点可相减，只作为后续周期的基准。 */
        s_last_capture_tick = captured_tick;
        s_has_first_capture = true;
        return;
    }

    /* 写数组前同时检查本次目标数量和物理数组上限，防止任何越界。 */
    period_index = s_period_count;
    if ((period_index >= s_period_target) ||
        (period_index >= BSP_LOAD_PERIOD_MAX_COUNT))
    {
        s_capture_active = false;
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
        return;
    }

    /* uint32_t无符号减法可正确处理TIM5计数回绕。 */
    s_periods[period_index] = captured_tick - s_last_capture_tick;
    s_last_capture_tick = captured_tick;
    period_index++;
    s_period_count = period_index;

    if (period_index >= s_period_target)
    {
        s_capture_active = false;
        __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
    }
}
