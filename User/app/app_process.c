#include "header.h"
#include "app_process.h"

#include "alog_measure.h"
#include "bsp_board.h"
#include "bsp_flash.h"
#include "bsp_length.h"
#include "bsp_load.h"
#include "bsp_path.h"

//*********************************************************************************************************
extern UART_HandleTypeDef huart1; // HMI 控制串口
extern UART_HandleTypeDef huart3; // PC 调试串口
//*********************************************************************************************************

static uint8_t hmi_rx_buffer[1];          // HMI 单字节命令接收缓冲区
static char debug_buffer[160];            // PC串口调试发送缓冲区
static volatile uint8_t hmi_cmd_flag = 0; // 新命令标志位
static volatile uint8_t hmi_cmd_data = 0; // 最新命令字节
//*********************************************************************************************************
#define APP_MEASURE_TOTAL_TIMEOUT_MS (5000U)
#define APP_LENGTH_REQUIRED_VALID_COUNT (16U)
#define APP_LENGTH_MIN_VALID_COUNT (8U)
#define APP_LENGTH_MAX_ATTEMPTS (48U)
#define APP_LENGTH_TRIM_COUNT (2U)
#define APP_LENGTH_SINGLE_TIMEOUT_MS (20U)
#define APP_ADC_SAMPLE_COUNT (32U)
#define APP_PERIOD_SAMPLE_COUNT (16U)
#define APP_CAPTURE_TIMEOUT_MS (500U)
#define APP_HMI_CMD_BUFFER_SIZE (100U)
#define APP_HMI_TEXT_BUFFER_SIZE (64U)
#define APP_HMI_TERMINATOR_SIZE (3U)
#define APP_UART_TX_TIMEOUT_MS (100U)
#define APP_RESISTANCE_CANDIDATE_MIN_OHM (5.0f)
#define APP_RESISTANCE_CANDIDATE_MAX_OHM (50.0f)
/* 已确认的板上分压参考电阻标称值；实测标定值仍以Flash记录为准。 */
#define APP_FACTORY_RESISTANCE_REFERENCE_OHM (62.0f)
#define APP_LOAD_CALIBRATION_MASK \
    (BSP_CAL_VALID_RESISTANCE | BSP_CAL_VALID_CAPACITANCE)
#define APP_ALL_PHYSICAL_CALIBRATION_MASK \
    (BSP_CAL_VALID_LENGTH | APP_LOAD_CALIBRATION_MASK)

/* 三类测量不会并发，共用缓冲区可避免为电赛单任务流程重复占用RAM。 */
typedef union
{
    int32_t length_raw[APP_LENGTH_REQUIRED_VALID_COUNT];
    uint16_t resistance_adc[APP_ADC_SAMPLE_COUNT];
    uint32_t capacitance_period[APP_PERIOD_SAMPLE_COUNT];
} app_measure_buffer_t;

typedef struct
{
    bool length_valid;
    float length_m;
    bool open_period_valid;
    uint32_t open_period_ticks;
} app_measure_context_t;

typedef enum
{
    APP_LOAD_KIND_NONE = 0,
    APP_LOAD_KIND_RESISTANCE,
    APP_LOAD_KIND_CAPACITANCE,
    APP_LOAD_KIND_OPEN
} app_load_kind_t;

static app_measure_buffer_t s_measure_buffer;
static bsp_calibration_data_t g_calibration;
static app_measure_context_t g_measure_context;
static bsp_length_result_t g_length_init_result;

/*
 * 工厂默认值只填入已经确认的62欧分压参考电阻。
 * valid_mask和其余标定项仍保持0，分压方向、增益、偏移及电缆补偿
 * 必须经原理图确认或真实测量后，才允许显式写入Flash并启用物理量输出。
 */
static void app_set_factory_calibration(void)
{
    memset(&g_calibration, 0, sizeof(g_calibration));
    g_calibration.resistance_reference_ohm =
        APP_FACTORY_RESISTANCE_REFERENCE_OHM;
}

static uint32_t app_remaining_ms(uint32_t start_ms)
{
    uint32_t elapsed_ms = bsp_now_ms() - start_ms;

    return (elapsed_ms < APP_MEASURE_TOTAL_TIMEOUT_MS) ? (APP_MEASURE_TOTAL_TIMEOUT_MS - elapsed_ms) : 0U;
}

static bool app_has_calibration(uint32_t required_mask)
{
    return (g_calibration.valid_mask & required_mask) == required_mask;
}

static void app_sort_i32(int32_t *samples, uint16_t count)
{
    int32_t value;
    uint16_t index;
    uint16_t insert_index;

    /* 样本最多16个，插入排序代码短且无需额外RAM。 */
    for (index = 1U; index < count; index++)
    {
        value = samples[index];
        insert_index = index;
        while ((insert_index > 0U) &&
               (samples[insert_index - 1U] > value))
        {
            samples[insert_index] = samples[insert_index - 1U];
            insert_index--;
        }
        samples[insert_index] = value;
    }
}

static void hmi_set_text(const char *object_name, const char *text)
{
    char command[APP_HMI_TEXT_BUFFER_SIZE];
    int length;

    if ((object_name == NULL) || (text == NULL))
    {
        return;
    }

    length = snprintf(command,
                      sizeof(command),
                      "%s.txt=\"%s\"",
                      object_name,
                      text);
    if ((length > 0) && ((size_t)length < sizeof(command)))
    {
        HMI_Send_Cmd(command);
    }
}

//=========================================================================================================
// 1. 基础功能函数
//=========================================================================================================
/**
 * @name HMI_Process_Init
 * @brief 启动应用层接收与测量任务框架
 */
void HMI_Process_Init(bsp_length_result_t length_init_result)
{
    bsp_flash_result_t flash_result;

    app_set_factory_calibration();
    memset(&g_measure_context, 0, sizeof(g_measure_context));
    g_length_init_result = length_init_result;

    flash_result = bsp_flash_load(&g_calibration);
    if (flash_result == BSP_FLASH_RESULT_OK)
    {
        Debug_printf("[FLASH] calibration loaded, valid_mask=0x%08lX\r\n",
                     (unsigned long)g_calibration.valid_mask);
    }
    else
    {
        /*
         * 擦除态或损坏记录都使用valid_mask=0的安全RAM副本。
         * 原始采集仍可运行，且初始化阶段不会自动擦写Flash。
         */
        app_set_factory_calibration();
        Debug_printf("[FLASH] no valid calibration, result=%u, raw mode\r\n",
                     (unsigned int)flash_result);
    }

    /*
     * 兼容旧版Flash记录：CRC正确但参考电阻字段为空、为负或为NaN时，
     * 仅在RAM中回退到已确认的62欧，不自动擦写用户的标定扇区。
     */
    if (!(g_calibration.resistance_reference_ohm > 0.0f))
    {
        g_calibration.resistance_reference_ohm =
            APP_FACTORY_RESISTANCE_REFERENCE_OHM;
        Debug_printf("[CAL] invalid resistance reference, use 62 ohm\r\n");
    }

    Debug_printf("[GP22] init result=%u\r\n",
                 (unsigned int)g_length_init_result);
    (void)HAL_UART_Receive_IT(&huart1, hmi_rx_buffer, 1);

    hmi_set_text("len", "--");
    hmi_set_text("kind", "--");
    hmi_set_text("unknow", "--");
    if (g_length_init_result != BSP_LENGTH_RESULT_OK)
    {
        hmi_set_text("status", "GP22 Error");
    }
    else if (!app_has_calibration(APP_ALL_PHYSICAL_CALIBRATION_MASK))
    {
        hmi_set_text("status", "Need Cal");
    }
    else
    {
        hmi_set_text("status", "Ready");
    }
}

/**
 * @name    HMI_Send_Cmd
 * @brief   向HMI串口屏发送原始指令
 */
void HMI_Send_Cmd(const char *cmd_string)
{
    char cmd_buffer[APP_HMI_CMD_BUFFER_SIZE];
    int len;
    uint16_t tx_length;

    if (cmd_string == NULL)
    {
        return;
    }

    /*
     * 预留结尾'\0'和HMI要求的3个0xFF，超长命令只截断正文，
     * 不会把snprintf返回的“理论长度”误当成实际缓冲区长度发送。
     */
    len = snprintf(cmd_buffer,
                   sizeof(cmd_buffer),
                   "%.*s\xff\xff\xff",
                   (int)(sizeof(cmd_buffer) -
                         APP_HMI_TERMINATOR_SIZE - 1U),
                   cmd_string);
    if (len > 0)
    {
        tx_length = ((size_t)len < sizeof(cmd_buffer)) ? (uint16_t)len : (uint16_t)(sizeof(cmd_buffer) - 1U);
        (void)HAL_UART_Transmit(&huart1,
                                (uint8_t *)cmd_buffer,
                                tx_length,
                                APP_UART_TX_TIMEOUT_MS);
    }
}

/**
 * @name    Debug_printf
 * @brief   PC串口调试打印
 */
void Debug_printf(const char *text, ...)
{
    va_list args;
    int len;
    uint16_t tx_length;

    if (text == NULL)
    {
        return;
    }

    va_start(args, text);
    len = vsnprintf(debug_buffer, sizeof(debug_buffer), text, args);
    va_end(args);

    if (len > 0)
    {
        tx_length = ((size_t)len < sizeof(debug_buffer)) ? (uint16_t)len : (uint16_t)(sizeof(debug_buffer) - 1U);
        (void)HAL_UART_Transmit(&huart3,
                                (uint8_t *)debug_buffer,
                                tx_length,
                                APP_UART_TX_TIMEOUT_MS);
    }
}
//=========================================================================================================
// 2. app层辅助任务函数
//=========================================================================================================
bsp_flash_result_t App_Calibration_Save(void)
{
    bsp_flash_result_t result;

    /*
     * 此入口只应由“用户明确保存标定”流程在主循环调用。
     * 先断开全部继电器，Flash擦写失败也保留g_calibration的RAM内容。
     */
    bsp_path_off();
    result = bsp_flash_save(&g_calibration);
    Debug_printf("[FLASH] calibration save result=%u\r\n",
                 (unsigned int)result);
    return result;
}

static void app_run_length_measurement(void)
{
    uint32_t start_ms = bsp_now_ms();
    uint32_t remaining_ms;
    uint32_t single_timeout_ms;
    uint32_t capture_timeout_ms;
    uint32_t elapsed_ms;
    uint32_t period_minimum;
    uint32_t period_maximum;
    uint32_t period_average;
    uint32_t capture_tick_hz;
    int32_t raw_value;
    int32_t raw_minimum = 0;
    int32_t raw_maximum = 0;
    int32_t raw_average = 0;
    float measured_length_m;
    uint16_t status_register;
    uint16_t attempt_count = 0U;
    uint16_t valid_count = 0U;
    uint16_t average_count;
    bsp_length_result_t result = BSP_LENGTH_RESULT_NOT_READY;
    bsp_load_result_t period_result = BSP_LOAD_RESULT_NOT_READY;
    bool fatal_error = false;
    bool timed_out = false;
    bool calibration_missing = false;
    bool success = false;
    char display_text[32];

    Debug_printf("[LENGTH] 正在检测\r\n");
    hmi_set_text("status", "Length...");
    hmi_set_text("len", "--");
    hmi_set_text("kind", "--");
    hmi_set_text("unknow", "--");

    /*
     * 上电初始化或上次SPI故障可能让GP22处于未就绪状态。
     * 每次按A只重试一次初始化；失败时不会进入48次采样循环。
     */
    if (g_length_init_result != BSP_LENGTH_RESULT_OK)
    {
        g_length_init_result = bsp_length_init();
        Debug_printf("[GP22] retry init result=%u\r\n",
                     (unsigned int)g_length_init_result);
        if (g_length_init_result != BSP_LENGTH_RESULT_OK)
        {
            result = g_length_init_result;
            fatal_error = true;
            goto cleanup;
        }
    }

    if (!bsp_path_select(BSP_PATH_LENGTH))
    {
        Debug_printf("[LENGTH] path error\r\n");
        fatal_error = true;
        goto cleanup;
    }

    while ((attempt_count < APP_LENGTH_MAX_ATTEMPTS) &&
           (valid_count < APP_LENGTH_REQUIRED_VALID_COUNT))
    {
        remaining_ms = app_remaining_ms(start_ms);
        if (remaining_ms == 0U)
        {
            result = BSP_LENGTH_RESULT_TIMEOUT;
            timed_out = true;
            break;
        }

        single_timeout_ms =
            (remaining_ms < APP_LENGTH_SINGLE_TIMEOUT_MS) ?
            remaining_ms : APP_LENGTH_SINGLE_TIMEOUT_MS;
        attempt_count++;
        result = bsp_length_measure_raw(&raw_value,
                                        &status_register,
                                        single_timeout_ms);
        if (result == BSP_LENGTH_RESULT_OK)
        {
            s_measure_buffer.length_raw[valid_count] = raw_value;
            valid_count++;
        }
        else if ((result == BSP_LENGTH_RESULT_NOT_READY) ||
                 (result == BSP_LENGTH_RESULT_SPI_ERROR) ||
                 (result == BSP_LENGTH_RESULT_INVALID_PARAM))
        {
            /*
             * 这三类错误重试原始测量没有意义。SPI/NOT_READY会在下一次
             * 按A时触发一次完整bsp_length_init，避免永久锁死。
             */
            g_length_init_result = result;
            fatal_error = true;
            break;
        }
    }

    elapsed_ms = bsp_now_ms() - start_ms;
    if (elapsed_ms >= APP_MEASURE_TOTAL_TIMEOUT_MS)
    {
        result = BSP_LENGTH_RESULT_TIMEOUT;
        timed_out = true;
    }

    if (valid_count > 0U)
    {
        app_sort_i32(s_measure_buffer.length_raw, valid_count);
        raw_minimum = s_measure_buffer.length_raw[0];
        raw_maximum = s_measure_buffer.length_raw[valid_count - 1U];
    }
    if ((valid_count >= APP_LENGTH_MIN_VALID_COUNT) &&
        (!fatal_error) && (!timed_out))
    {
        average_count =
            valid_count - (2U * APP_LENGTH_TRIM_COUNT);
        if (!alog_average_i32(
                &s_measure_buffer.length_raw[APP_LENGTH_TRIM_COUNT],
                average_count,
                &raw_average))
        {
            fatal_error = true;
        }
    }

    elapsed_ms = bsp_now_ms() - start_ms;
    Debug_printf("[LENGTH] attempt_count=%u, valid_count=%u, raw_min=%ld, raw_max=%ld, raw_average=%ld, elapsed_ms=%lu\r\n",
                 (unsigned int)attempt_count,
                 (unsigned int)valid_count,
                 (long)raw_minimum,
                 (long)raw_maximum,
                 (long)raw_average,
                 (unsigned long)elapsed_ms);

    if ((valid_count < APP_LENGTH_MIN_VALID_COUNT) ||
        fatal_error || timed_out)
    {
        goto cleanup;
    }

    Debug_printf("CAL,L,%ld,%u,%lu\r\n",
                 (long)raw_average,
                 (unsigned int)valid_count,
                 (unsigned long)elapsed_ms);

    if (!app_has_calibration(BSP_CAL_VALID_LENGTH))
    {
        calibration_missing = true;
        goto cleanup;
    }
    if (!alog_length_from_raw(raw_average,
                              &g_calibration,
                              &measured_length_m))
    {
        Debug_printf("[LENGTH] calibrated result out of range\r\n");
        goto cleanup;
    }

    /*
     * 长度和开路周期必须成对更新。若基准捕获失败，保留上一次完整上下文，
     * 避免新长度配上旧开路周期后误判电容。
     */
    remaining_ms = app_remaining_ms(start_ms);
    if (remaining_ms == 0U)
    {
        timed_out = true;
        goto cleanup;
    }
    if (!bsp_path_select(BSP_PATH_CAPACITANCE))
    {
        Debug_printf("[LENGTH] open-period path error\r\n");
        goto cleanup;
    }

    remaining_ms = app_remaining_ms(start_ms);
    if (remaining_ms == 0U)
    {
        timed_out = true;
        goto cleanup;
    }
    capture_timeout_ms =
        (remaining_ms < APP_CAPTURE_TIMEOUT_MS) ?
        remaining_ms : APP_CAPTURE_TIMEOUT_MS;
    period_result = bsp_load_capture_periods(
        s_measure_buffer.capacitance_period,
        APP_PERIOD_SAMPLE_COUNT,
        capture_timeout_ms);
    if ((period_result != BSP_LOAD_RESULT_OK) ||
        (!alog_stats_u32(s_measure_buffer.capacitance_period,
                         APP_PERIOD_SAMPLE_COUNT,
                         &period_minimum,
                         &period_maximum,
                         &period_average)))
    {
        Debug_printf("[LENGTH] open-period failed result=%u\r\n",
                     (unsigned int)period_result);
        goto cleanup;
    }

    capture_tick_hz = bsp_load_get_capture_tick_hz();
    Debug_printf("NE555,min=%lu,max=%lu,avg=%lu,span=%lu,tick_hz=%lu\r\n",
                 (unsigned long)period_minimum,
                 (unsigned long)period_maximum,
                 (unsigned long)period_average,
                 (unsigned long)(period_maximum - period_minimum),
                 (unsigned long)capture_tick_hz);

    if (app_remaining_ms(start_ms) == 0U)
    {
        timed_out = true;
        goto cleanup;
    }

    g_measure_context.length_m = measured_length_m;
    g_measure_context.length_valid = true;
    g_measure_context.open_period_ticks = period_average;
    g_measure_context.open_period_valid = true;
    success = true;

cleanup:
    bsp_load_init();
    bsp_path_off();
    elapsed_ms = bsp_now_ms() - start_ms;
    if (success)
    {
        (void)snprintf(display_text,
                       sizeof(display_text),
                       "%.2f m",
                       (double)g_measure_context.length_m);
        hmi_set_text("len", display_text);
        hmi_set_text("status", "Hold");
        Debug_printf("[LENGTH] done length=%.3f m, open_tick=%lu, elapsed_ms=%lu\r\n",
                     (double)g_measure_context.length_m,
                     (unsigned long)g_measure_context.open_period_ticks,
                     (unsigned long)elapsed_ms);
    }
    else if (timed_out ||
             (elapsed_ms >= APP_MEASURE_TOTAL_TIMEOUT_MS))
    {
        hmi_set_text("len", "--");
        hmi_set_text("status", "Timeout");
        Debug_printf("[LENGTH] timeout result=%u, elapsed_ms=%lu\r\n",
                     (unsigned int)result,
                     (unsigned long)elapsed_ms);
    }
    else if ((g_length_init_result != BSP_LENGTH_RESULT_OK) &&
             fatal_error)
    {
        hmi_set_text("len", "--");
        hmi_set_text("status", "GP22 Error");
        Debug_printf("[LENGTH] GP22 error result=%u, elapsed_ms=%lu\r\n",
                     (unsigned int)result,
                     (unsigned long)elapsed_ms);
    }
    else if (calibration_missing)
    {
        hmi_set_text("len", "--");
        hmi_set_text("status", "Need Cal");
        Debug_printf("[LENGTH] calibration required, elapsed_ms=%lu\r\n",
                     (unsigned long)elapsed_ms);
    }
    else
    {
        hmi_set_text("len", "--");
        hmi_set_text("status", "Length Error");
        Debug_printf("[LENGTH] failed result=%u, valid_count=%u, elapsed_ms=%lu\r\n",
                     (unsigned int)result,
                     (unsigned int)valid_count,
                     (unsigned long)elapsed_ms);
    }
}

static void app_run_load_measurement(void)
{
    uint32_t start_ms = bsp_now_ms();
    uint32_t remaining_ms;
    uint32_t capture_timeout_ms;
    uint32_t elapsed_ms;
    uint32_t period_minimum;
    uint32_t period_maximum;
    uint32_t period_average;
    uint32_t capture_tick_hz;
    uint32_t delta_ticks;
    uint16_t adc_minimum;
    uint16_t adc_maximum;
    uint16_t adc_average;
    float resistance_ohm = 0.0f;
    float capacitance_pf = 0.0f;
    bsp_load_result_t adc_result = BSP_LOAD_RESULT_NOT_READY;
    bsp_load_result_t period_result = BSP_LOAD_RESULT_NOT_READY;
    app_load_kind_t load_kind = APP_LOAD_KIND_NONE;
    bool timed_out = false;
    bool calibration_missing = false;
    bool measure_length_first = false;
    bool success = false;
    char display_text[32];

    Debug_printf("[LOAD] 正在检测\r\n");
    hmi_set_text("status", "Load...");
    hmi_set_text("kind", "--");
    hmi_set_text("unknow", "--");

    if ((!g_measure_context.length_valid) ||
        (!g_measure_context.open_period_valid))
    {
        measure_length_first = true;
        goto cleanup;
    }

    if (!bsp_path_select(BSP_PATH_RESISTANCE))
    {
        Debug_printf("[LOAD] resistance path error\r\n");
        goto cleanup;
    }

    adc_result = bsp_load_read_adc(s_measure_buffer.resistance_adc,
                                   APP_ADC_SAMPLE_COUNT);
    if ((adc_result != BSP_LOAD_RESULT_OK) ||
        (!alog_stats_u16(s_measure_buffer.resistance_adc,
                         APP_ADC_SAMPLE_COUNT,
                         &adc_minimum,
                         &adc_maximum,
                         &adc_average)))
    {
        Debug_printf("[LOAD] ADC failed result=%u\r\n",
                     (unsigned int)adc_result);
        goto cleanup;
    }

    Debug_printf("ADC,min=%u,max=%u,avg=%u,span=%u\r\n",
                 (unsigned int)adc_minimum,
                 (unsigned int)adc_maximum,
                 (unsigned int)adc_average,
                 (unsigned int)(adc_maximum - adc_minimum));
    Debug_printf("CAL,R,%u,%u,%u,%.3f\r\n",
                 (unsigned int)adc_minimum,
                 (unsigned int)adc_maximum,
                 (unsigned int)adc_average,
                 (double)g_measure_context.length_m);

    if (app_remaining_ms(start_ms) == 0U)
    {
        timed_out = true;
        goto cleanup;
    }

    if (alog_resistance_from_adc(adc_average,
                                 g_measure_context.length_m,
                                 &g_calibration,
                                 &resistance_ohm) &&
        (resistance_ohm >= APP_RESISTANCE_CANDIDATE_MIN_OHM) &&
        (resistance_ohm <= APP_RESISTANCE_CANDIDATE_MAX_OHM))
    {
        load_kind = APP_LOAD_KIND_RESISTANCE;
        success = true;
        goto cleanup;
    }

    /* ADC未形成5~50 ohm候选时，再采集NE555区分电容和开路。 */
    remaining_ms = app_remaining_ms(start_ms);
    if (remaining_ms == 0U)
    {
        timed_out = true;
        goto cleanup;
    }
    if (!bsp_path_select(BSP_PATH_CAPACITANCE))
    {
        Debug_printf("[LOAD] capacitance path error\r\n");
        goto cleanup;
    }

    remaining_ms = app_remaining_ms(start_ms);
    if (remaining_ms == 0U)
    {
        timed_out = true;
        goto cleanup;
    }
    capture_timeout_ms =
        (remaining_ms < APP_CAPTURE_TIMEOUT_MS) ?
        remaining_ms : APP_CAPTURE_TIMEOUT_MS;
    period_result = bsp_load_capture_periods(
        s_measure_buffer.capacitance_period,
        APP_PERIOD_SAMPLE_COUNT,
        capture_timeout_ms);
    if ((period_result != BSP_LOAD_RESULT_OK) ||
        (!alog_stats_u32(s_measure_buffer.capacitance_period,
                         APP_PERIOD_SAMPLE_COUNT,
                         &period_minimum,
                         &period_maximum,
                         &period_average)))
    {
        Debug_printf("[LOAD] NE555 failed result=%u\r\n",
                     (unsigned int)period_result);
        goto cleanup;
    }

    capture_tick_hz = bsp_load_get_capture_tick_hz();
    Debug_printf("NE555,min=%lu,max=%lu,avg=%lu,span=%lu,tick_hz=%lu\r\n",
                 (unsigned long)period_minimum,
                 (unsigned long)period_maximum,
                 (unsigned long)period_average,
                 (unsigned long)(period_maximum - period_minimum),
                 (unsigned long)capture_tick_hz);

    delta_ticks =
        (period_average > g_measure_context.open_period_ticks) ?
        (period_average - g_measure_context.open_period_ticks) : 0U;
    Debug_printf("CAL,C,%lu,%lu,%lu,%lu\r\n",
                 (unsigned long)g_measure_context.open_period_ticks,
                 (unsigned long)period_average,
                 (unsigned long)delta_ticks,
                 (unsigned long)capture_tick_hz);

    if (app_remaining_ms(start_ms) == 0U)
    {
        timed_out = true;
        goto cleanup;
    }

    /*
     * 缺任一负载标定时仍完成两条原始链路和CAL日志，但不输出虚假类型。
     * resistor_detect_threshold/capacitor_detect_threshold本阶段不使用，
     * 因为仓库没有定义其单位，分类直接采用题目候选范围。
     */
    if (!app_has_calibration(APP_LOAD_CALIBRATION_MASK))
    {
        calibration_missing = true;
        goto cleanup;
    }

    if (alog_capacitance_from_period(
            g_measure_context.open_period_ticks,
            period_average,
            &g_calibration,
            &capacitance_pf))
    {
        load_kind = APP_LOAD_KIND_CAPACITANCE;
    }
    else
    {
        load_kind = APP_LOAD_KIND_OPEN;
    }
    success = true;

cleanup:
    bsp_load_init();
    bsp_path_off();
    elapsed_ms = bsp_now_ms() - start_ms;
    if (success && (elapsed_ms >= APP_MEASURE_TOTAL_TIMEOUT_MS))
    {
        success = false;
        timed_out = true;
    }

    if (success)
    {
        if (load_kind == APP_LOAD_KIND_RESISTANCE)
        {
            (void)snprintf(display_text,
                           sizeof(display_text),
                           "%.1f ohm",
                           (double)resistance_ohm);
            hmi_set_text("kind", "R");
            hmi_set_text("unknow", display_text);
        }
        else if (load_kind == APP_LOAD_KIND_CAPACITANCE)
        {
            (void)snprintf(display_text,
                           sizeof(display_text),
                           "%.0f pF",
                           (double)capacitance_pf);
            hmi_set_text("kind", "C");
            hmi_set_text("unknow", display_text);
        }
        else
        {
            hmi_set_text("kind", "OPEN");
            hmi_set_text("unknow", "--");
        }
        hmi_set_text("status", "Hold");
    }
    else if (measure_length_first)
    {
        hmi_set_text("kind", "--");
        hmi_set_text("unknow", "--");
        hmi_set_text("status", "Measure L First");
    }
    else if (timed_out ||
             (elapsed_ms >= APP_MEASURE_TOTAL_TIMEOUT_MS))
    {
        hmi_set_text("kind", "ERROR");
        hmi_set_text("unknow", "--");
        hmi_set_text("status", "Timeout");
    }
    else if (calibration_missing)
    {
        hmi_set_text("kind", "--");
        hmi_set_text("unknow", "--");
        hmi_set_text("status", "Need Cal");
    }
    else
    {
        hmi_set_text("kind", "ERROR");
        hmi_set_text("unknow", "--");
        hmi_set_text("status", "Load Error");
    }

    Debug_printf("[LOAD] done success=%u, kind=%u, adc_result=%u, period_result=%u, elapsed_ms=%lu\r\n",
                 success ? 1U : 0U,
                 (unsigned int)load_kind,
                 (unsigned int)adc_result,
                 (unsigned int)period_result,
                 (unsigned long)elapsed_ms);
}
//=========================================================================================================
// 3. 应用任务函数
//=========================================================================================================
/**
 * @name   Task_Button_Poll
 * @brief  按键响应函数，放入主循环轮询中，按键触发后修改状态机状态
 */
static void Task_Button_Poll(void)
{
    uint8_t cmd;

    if (hmi_cmd_flag == 0U)
    {
        return;
    }

    __disable_irq(); // 屏蔽中断
    cmd = hmi_cmd_data;
    hmi_cmd_flag = 0;
    __enable_irq(); // 恢复中断屏蔽

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    Debug_printf("[KEY] RX cmd='%c' hex=0x%02X\r\n", cmd, cmd);
    if (cmd == 'A')
    {
        Debug_printf("[KEY]cmd=A| xxxx \r\n");
        app_run_length_measurement();
    }
    else if (cmd == 'B')
    {
        Debug_printf("[KEY]cmd=B| xxxx \r\n");
        app_run_load_measurement();
    }
    else
    {
        Debug_printf("[KEY] Unknown cmd|use 'A'=XXXX, 'B'=YYYY.\r\n");
    }
}
//=========================================================================================================
// 4. 主轮询整合
//=========================================================================================================
/**
 * @name   App_Main_Process_Poll
 * @brief  放在main.c的while(1)中，统筹调度所有应用任务
 */
void App_Main_Process_Poll(void)
{
    Task_Button_Poll();
}
//=========================================================================================================
// 5. 中断回调
//=========================================================================================================
/**
 * @brief USART接收完成回调，只保存命令并重启接收
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        hmi_cmd_data = hmi_rx_buffer[0];
        hmi_cmd_flag = 1;
        HAL_UART_Receive_IT(huart, hmi_rx_buffer, 1);
    }
}

/**
 * @brief USART1发生噪声、帧或溢出错误后恢复单字节中断接收。
 * @note 回调运行在中断上下文，只重启接收，不执行日志或Flash操作。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        (void)HAL_UART_Receive_IT(huart, hmi_rx_buffer, 1);
    }
}
