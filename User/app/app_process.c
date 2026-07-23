#include "app_process.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "alog_measure.h"
#include "bsp_length.h"
#include "bsp_load.h"
#include "bsp_path.h"
#include "usart.h"

#define APP_UART_TX_TIMEOUT_MS       (100U)
#define APP_HMI_COMMAND_BUFFER_SIZE  (100U)
#define APP_HMI_TEXT_BUFFER_SIZE     (64U)
#define APP_DEBUG_BUFFER_SIZE        (192U)

/* 最坏长度等待2 s，电容捕获等待1 s，两类任务均有明确的5 s内软件边界。 */
#define APP_LENGTH_ATTEMPTS          (100U)
#define APP_LENGTH_MIN_SUCCESS       (10U)
#define APP_LENGTH_TIMEOUT_MS        (20U)
#define APP_ADC_SAMPLE_COUNT         (100U)
#define APP_PERIOD_SAMPLE_COUNT      (32U)
#define APP_PERIOD_TIMEOUT_MS        (1000U)

static uint8_t g_hmi_rx_buffer[1];
static char g_debug_buffer[APP_DEBUG_BUFFER_SIZE];
static volatile uint8_t g_hmi_cmd_flag;
static volatile uint8_t g_hmi_cmd_data;
static float g_last_length_cm;
static bool g_length_valid;
/* 样本缓冲区放在静态区，避免STM32默认1 KB栈被大数组占满。 */
static uint32_t g_length_samples[APP_LENGTH_ATTEMPTS];
static uint16_t g_adc_samples[APP_ADC_SAMPLE_COUNT];
static uint32_t g_period_samples[APP_PERIOD_SAMPLE_COUNT];

static uint16_t text_length_clamped(int formatted_length,
                                    size_t buffer_size)
{
    if ((formatted_length <= 0) || (buffer_size == 0U))
    {
        return 0U;
    }
    if ((size_t)formatted_length >= buffer_size)
    {
        return (uint16_t)(buffer_size - 1U);
    }
    return (uint16_t)formatted_length;
}

void HMI_Send_Cmd(const char *cmd_string)
{
    char command[APP_HMI_COMMAND_BUFFER_SIZE];
    int formatted_length;
    uint16_t transmit_length;

    if (cmd_string == NULL)
    {
        return;
    }

    formatted_length = snprintf(command,
                                sizeof(command),
                                "%s\xFF\xFF\xFF",
                                cmd_string);
    transmit_length = text_length_clamped(formatted_length,
                                          sizeof(command));
    if (transmit_length > 0U)
    {
        (void)HAL_UART_Transmit(&huart1,
                                (uint8_t *)command,
                                transmit_length,
                                APP_UART_TX_TIMEOUT_MS);
    }
}

void Debug_printf(const char *text, ...)
{
    va_list arguments;
    int formatted_length;
    uint16_t transmit_length;

    if (text == NULL)
    {
        return;
    }

    va_start(arguments, text);
    formatted_length = vsnprintf(g_debug_buffer,
                                 sizeof(g_debug_buffer),
                                 text,
                                 arguments);
    va_end(arguments);
    transmit_length = text_length_clamped(formatted_length,
                                          sizeof(g_debug_buffer));
    if (transmit_length > 0U)
    {
        (void)HAL_UART_Transmit(&huart3,
                                (uint8_t *)g_debug_buffer,
                                transmit_length,
                                APP_UART_TX_TIMEOUT_MS);
    }
}

static void HMI_SetText(const char *name, const char *text)
{
    char command[APP_HMI_TEXT_BUFFER_SIZE];
    int formatted_length;

    if ((name == NULL) || (text == NULL))
    {
        return;
    }

    /* Nextion文本控件统一采用name.txt="text"，结束符由HMI_Send_Cmd追加。 */
    formatted_length = snprintf(command,
                                sizeof(command),
                                "%s.txt=\"%s\"",
                                name,
                                text);
    if ((formatted_length > 0) &&
        ((size_t)formatted_length < sizeof(command)))
    {
        HMI_Send_Cmd(command);
    }
}

static void app_set_error(void)
{
    HMI_SetText("kind", "ERROR");
    HMI_SetText("para", "--");
    HMI_SetText("status", "Error");
}

static void app_min_max_u16(const uint16_t *data,
                            uint16_t count,
                            uint16_t *minimum,
                            uint16_t *maximum)
{
    uint16_t index;
    uint16_t min_value = data[0];
    uint16_t max_value = data[0];

    for (index = 1U; index < count; index++)
    {
        if (data[index] < min_value)
        {
            min_value = data[index];
        }
        if (data[index] > max_value)
        {
            max_value = data[index];
        }
    }
    *minimum = min_value;
    *maximum = max_value;
}

static void app_min_max_u32(const uint32_t *data,
                            uint16_t count,
                            uint32_t *minimum,
                            uint32_t *maximum)
{
    uint16_t index;
    uint32_t min_value = data[0];
    uint32_t max_value = data[0];

    for (index = 1U; index < count; index++)
    {
        if (data[index] < min_value)
        {
            min_value = data[index];
        }
        if (data[index] > max_value)
        {
            max_value = data[index];
        }
    }
    *minimum = min_value;
    *maximum = max_value;
}

static void app_log_length_reference(uint32_t raw, float length_cm)
{
    float fixed_value;
    float time_ns;

    fixed_value = (float)(int16_t)(raw >> 16U);
    fixed_value += (float)(raw & 0xFFFFU) / 65536.0f;
    time_ns = fixed_value / 0.002f;
    /* 同时保留raw和中间量，便于判断问题来自GP22还是参考公式。 */
    Debug_printf("[L_REF_SIM] fixed=%.6f time_ns=%.3f length_cm=%.2f\r\n",
                 (double)fixed_value,
                 (double)time_ns,
                 (double)length_cm);
}

static void app_log_resistance_reference(uint16_t adc_average,
                                         float cable_length_cm,
                                         float load_resistance)
{
    float voltage_raw;
    float voltage_sim;
    float denominator;
    float resistance_calculated = -1.0f;
    float cable_resistance;

    voltage_raw = (float)adc_average * 3.3f / 4096.0f;
    voltage_sim = (voltage_raw * 1.0072f) - 0.0007f;
    denominator = 3.3f - voltage_sim;
    if ((voltage_sim > 0.0f) && (denominator > 0.0f))
    {
        resistance_calculated = 62.0f * voltage_sim / denominator;
    }
    cable_resistance = (0.0014f * cable_length_cm) + 0.0328f;
    Debug_printf("[R_REF_SIM] r_calc=%.3f cable_r=%.3f r_load=%.3f\r\n",
                 (double)resistance_calculated,
                 (double)cable_resistance,
                 (double)load_resistance);
}

static void app_log_capacitance_reference(uint32_t period_average,
                                          float cable_length_cm,
                                          float load_capacitance)
{
    float frequency_raw;
    float frequency_sim;
    float total_calculated;
    float total_sim;
    float cable_capacitance;

    if (period_average == 0U)
    {
        Debug_printf("[C_REF_SIM] invalid_period\r\n");
        return;
    }

    frequency_raw = TIM5_CAPTURE_TICK_HZ / (float)period_average;
    frequency_sim = (frequency_raw * 1.0048f) - 0.0501f;
    if (frequency_sim <= 0.0f)
    {
        Debug_printf("[C_REF_SIM] invalid_frequency\r\n");
        return;
    }

    total_calculated = 1000000.0f * 1.442695f /
                       (0.01f + (2.0f * 1.0f)) /
                       frequency_sim;
    total_sim = 1.003054914f *
                (total_calculated - 36.6104f);
    cable_capacitance = (0.9527f * cable_length_cm) + 1.4487f;
    Debug_printf("[C_REF_SIM] freq=%.3f total=%.3f cable=%.3f load=%.3f\r\n",
                 (double)frequency_sim,
                 (double)total_sim,
                 (double)cable_capacitance,
                 (double)load_capacitance);
}

static void App_Task_Length(void)
{
    uint32_t raw = 0U;
    uint32_t raw_average = 0U;
    uint32_t start_ms = HAL_GetTick();
    uint16_t status = 0U;
    uint16_t success_count = 0U;
    uint16_t failure_count = 0U;
    uint16_t attempt;
    float length_cm = -1.0f;
    bool success = false;
    char display[32];

    HMI_SetText("status", "Measuring");
    HMI_SetText("kind", "--");
    HMI_SetText("para", "--");

    /* 每次A命令都重新初始化GP22，使上一次SPI/超时故障不锁死后续测量。 */
    if ((!bsp_path_select(BSP_PATH_LENGTH)) ||
        (!bsp_length_init()))
    {
        Debug_printf("[L_RAW] init_or_path_failed\r\n");
        goto cleanup;
    }

    for (attempt = 0U; attempt < APP_LENGTH_ATTEMPTS; attempt++)
    {
        /* 失败样本跳过；完整raw只打印前三条，避免串口日志拖慢任务。 */
        if (bsp_length_measure(&raw,
                               &status,
                               APP_LENGTH_TIMEOUT_MS))
        {
            g_length_samples[success_count] = raw;
            success_count++;
            if (success_count <= 3U)
            {
                Debug_printf("[L_RAW] ok=1 raw=0x%08lX status=0x%04X\r\n",
                             (unsigned long)raw,
                             (unsigned int)status);
            }
        }
        else
        {
            failure_count++;
            Debug_printf("[L_RAW] fail=%u\r\n",
                         (unsigned int)failure_count);
        }
    }

    if (success_count < APP_LENGTH_MIN_SUCCESS)
    {
        Debug_printf("[L_RAW] insufficient=%u/%u\r\n",
                     (unsigned int)success_count,
                     (unsigned int)APP_LENGTH_ATTEMPTS);
        goto cleanup;
    }

    /* 只有达到最少成功数后才发布新的“最近长度”。 */
    raw_average = alog_average_tdc(g_length_samples, success_count);
    length_cm = alog_length_reference_cm(raw_average);
    Debug_printf("[L_RAW] ok=1 raw=0x%08lX status=0x%04X\r\n",
                 (unsigned long)raw_average,
                 (unsigned int)status);
    app_log_length_reference(raw_average, length_cm);
    if (!(length_cm >= 0.0f))
    {
        Debug_printf("[L_REF_SIM] invalid_negative_length\r\n");
        goto cleanup;
    }

    g_last_length_cm = length_cm;
    g_length_valid = true;
    (void)snprintf(display,
                   sizeof(display),
                   "%.2f cm",
                   (double)length_cm);
    HMI_SetText("len", display);
    HMI_SetText("status", "SIM Hold");
    success = true;

cleanup:
    /* 单一清理出口保证任何失败、超时或公式无效时继电器都会释放。 */
    bsp_path_off();
    Debug_printf("[L_SUM] success=%u samples=%u failures=%u raw_avg=0x%08lX length_cm=%.2f elapsed_ms=%lu\r\n",
                 success ? 1U : 0U,
                 (unsigned int)success_count,
                 (unsigned int)failure_count,
                 (unsigned long)raw_average,
                 (double)length_cm,
                 (unsigned long)(HAL_GetTick() - start_ms));
    if (!success)
    {
        app_set_error();
    }
}

static void App_Task_Load(void)
{
    uint32_t start_ms = HAL_GetTick();
    uint32_t period_minimum;
    uint32_t period_maximum;
    uint32_t period_average;
    uint16_t adc_minimum;
    uint16_t adc_maximum;
    uint16_t adc_average;
    float voltage_raw;
    float frequency_raw;
    float resistance_load = -1.0f;
    float capacitance_load = -1.0f;
    bool acquisition_failed = false;
    bool result_ready = false;
    char display[32];

    HMI_SetText("status", "Measuring");
    HMI_SetText("kind", "--");
    HMI_SetText("para", "--");

    /* B命令先尝试电阻；形成0~100 ohm候选后无需再接入NE555通路。 */
    if ((!bsp_path_select(BSP_PATH_RESISTANCE)) ||
        (!bsp_load_read_adc(g_adc_samples, APP_ADC_SAMPLE_COUNT)))
    {
        acquisition_failed = true;
        goto cleanup;
    }
    bsp_path_off();

    adc_average = alog_average_u16(g_adc_samples, APP_ADC_SAMPLE_COUNT);
    app_min_max_u16(g_adc_samples,
                    APP_ADC_SAMPLE_COUNT,
                    &adc_minimum,
                    &adc_maximum);
    voltage_raw = (float)adc_average * 3.3f / 4096.0f;
    Debug_printf("[R_RAW] min=%u max=%u avg=%u voltage=%.5f\r\n",
                 (unsigned int)adc_minimum,
                 (unsigned int)adc_maximum,
                 (unsigned int)adc_average,
                 (double)voltage_raw);

    if (g_length_valid)
    {
        resistance_load =
            alog_resistance_reference_ohm(adc_average,
                                          g_last_length_cm);
        app_log_resistance_reference(adc_average,
                                     g_last_length_cm,
                                     resistance_load);
        if ((resistance_load > 0.0f) &&
            (resistance_load <= 100.0f))
        {
            (void)snprintf(display,
                           sizeof(display),
                           "%.2f ohm",
                           (double)resistance_load);
            HMI_SetText("kind", "R");
            HMI_SetText("para", display);
            HMI_SetText("status", "SIM Hold");
            result_ready = true;
            goto cleanup;
        }
    }
    else
    {
        /* 没有长度也继续输出ADC和周期原始量，但禁止做电缆寄生补偿。 */
        Debug_printf("[REF_SIM] R skipped: measure length first\r\n");
    }

    if ((!bsp_path_select(BSP_PATH_CAPACITANCE)) ||
        (!bsp_load_capture_periods(g_period_samples,
                                   APP_PERIOD_SAMPLE_COUNT,
                                   APP_PERIOD_TIMEOUT_MS)))
    {
        acquisition_failed = true;
        goto cleanup;
    }
    bsp_path_off();

    period_average = alog_average_u32(g_period_samples,
                                      APP_PERIOD_SAMPLE_COUNT);
    app_min_max_u32(g_period_samples,
                    APP_PERIOD_SAMPLE_COUNT,
                    &period_minimum,
                    &period_maximum);
    if (period_average == 0U)
    {
        acquisition_failed = true;
        goto cleanup;
    }
    frequency_raw = TIM5_CAPTURE_TICK_HZ / (float)period_average;
    Debug_printf("[C_RAW] period_min=%lu period_max=%lu period_avg=%lu tick_hz=%.0f freq=%.3f\r\n",
                 (unsigned long)period_minimum,
                 (unsigned long)period_maximum,
                 (unsigned long)period_average,
                 (double)TIM5_CAPTURE_TICK_HZ,
                 (double)frequency_raw);

    if (!g_length_valid)
    {
        /* 原始链路已经验证完成，HMI提示用户先执行A命令后再分类。 */
        Debug_printf("[REF_SIM] C skipped: measure length first\r\n");
        HMI_SetText("status", "Measure L First");
        result_ready = true;
        goto cleanup;
    }

    capacitance_load =
        alog_capacitance_reference_pf(period_average,
                                      g_last_length_cm);
    app_log_capacitance_reference(period_average,
                                  g_last_length_cm,
                                  capacitance_load);
    if (capacitance_load >= 30.0f)
    {
        (void)snprintf(display,
                       sizeof(display),
                       "%.2f pF",
                       (double)capacitance_load);
        HMI_SetText("kind", "C");
        HMI_SetText("para", display);
    }
    else
    {
        HMI_SetText("kind", "OPEN");
        HMI_SetText("para", "--");
    }
    HMI_SetText("status", "SIM Hold");
    result_ready = true;

cleanup:
    bsp_path_off();
    Debug_printf("[LOAD_SUM] success=%u elapsed_ms=%lu\r\n",
                 result_ready ? 1U : 0U,
                 (unsigned long)(HAL_GetTick() - start_ms));
    if (acquisition_failed || (!result_ready))
    {
        app_set_error();
    }
}

void HMI_Process_Init(void)
{
    g_hmi_cmd_flag = 0U;
    g_hmi_cmd_data = 0U;
    g_last_length_cm = 0.0f;
    g_length_valid = false;

    HMI_SetText("status", "Ready");
    HMI_SetText("len", "--");
    HMI_SetText("kind", "--");
    HMI_SetText("para", "--");
    (void)HAL_UART_Receive_IT(&huart1, g_hmi_rx_buffer, 1U);
}

void App_Main_Process_Poll(void)
{
    uint8_t command;

    if (g_hmi_cmd_flag == 0U)
    {
        return;
    }

    /* 先原子地取走命令标志；任务期间收到的新命令留给下一轮主循环。 */
    __disable_irq();
    command = g_hmi_cmd_data;
    g_hmi_cmd_flag = 0U;
    __enable_irq();

    if (command == (uint8_t)'A')
    {
        App_Task_Length();
    }
    else if (command == (uint8_t)'B')
    {
        App_Task_Load();
    }
    else
    {
        Debug_printf("[KEY] unknown=0x%02X\r\n",
                     (unsigned int)command);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        /* UART ISR只保存单字节命令并立即重启接收，不直接执行测量。 */
        g_hmi_cmd_data = g_hmi_rx_buffer[0];
        g_hmi_cmd_flag = 1U;
        (void)HAL_UART_Receive_IT(huart, g_hmi_rx_buffer, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        (void)HAL_UART_Receive_IT(huart, g_hmi_rx_buffer, 1U);
    }
}
