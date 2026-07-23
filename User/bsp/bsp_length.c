#include "bsp_length.h"

#include <stddef.h>

#include "main.h"
#include "spi.h"
#include "tim.h"

/* GP22单字节控制命令以及结果/状态读取地址。 */
#define GP22_CMD_POWER_ON_RESET (0x50U)
#define GP22_CMD_INIT           (0x70U)
#define GP22_CMD_READ_RES0      (0xB0U)
#define GP22_CMD_READ_STATUS    (0xB4U)
#define GP22_CMD_READ_REG1_MSB  (0xB5U)

#define GP22_SPI_TIMEOUT_MS     (2U)

/*
 * 已由旧驱动验证的完整四字节配置命令：最高字节为写命令/寄存器地址，
 * 低24位为寄存器值。完成10 m/20 m原始量验证前不要修改这些固定值。
 */
static const uint32_t gp22_config_words[7] =
{
    0x80029660U,
    0x81010100U,
    0x82A00000U,
    0x83180000U,
    0x84200100U,
    0x85080000U,
    0x86000000U
};

/* initialized由主循环读写；irq_seen只由PE0 EXTI置位、主循环清除。 */
static bool g_gp22_initialized;
static volatile bool g_gp22_irq_seen;

static void gp22_set_ssn(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GP22_SSN_GPIO_Port, GP22_SSN_Pin, state);
}

static bool gp22_write(const uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef result;

    if ((data == NULL) || (size == 0U))
    {
        return false;
    }

    gp22_set_ssn(GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(&hspi1,
                              (uint8_t *)data,
                              size,
                              GP22_SPI_TIMEOUT_MS);
    gp22_set_ssn(GPIO_PIN_SET);
    return result == HAL_OK;
}

static bool gp22_command(uint8_t command)
{
    return gp22_write(&command, 1U);
}

static bool gp22_write_config_word(uint32_t word)
{
    uint8_t bytes[4];

    bytes[0] = (uint8_t)(word >> 24U);
    bytes[1] = (uint8_t)(word >> 16U);
    bytes[2] = (uint8_t)(word >> 8U);
    bytes[3] = (uint8_t)word;
    return gp22_write(bytes, (uint16_t)sizeof(bytes));
}

static bool gp22_read(uint8_t command, uint8_t *data, uint16_t size)
{
    uint8_t dummy[4] = {0U, 0U, 0U, 0U};
    HAL_StatusTypeDef result;

    if ((data == NULL) || (size == 0U) ||
        (size > (uint16_t)sizeof(dummy)))
    {
        return false;
    }

    gp22_set_ssn(GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(&hspi1,
                              &command,
                              1U,
                              GP22_SPI_TIMEOUT_MS);
    if (result == HAL_OK)
    {
        /* 继续保持SSN为低，通过dummy字节产生读取所需的SPI时钟。 */
        result = HAL_SPI_TransmitReceive(&hspi1,
                                         dummy,
                                         data,
                                         size,
                                         GP22_SPI_TIMEOUT_MS);
    }
    gp22_set_ssn(GPIO_PIN_SET);
    return result == HAL_OK;
}

static bool gp22_read_status(uint16_t *status)
{
    uint8_t bytes[2];

    if ((status == NULL) ||
        (!gp22_read(GP22_CMD_READ_STATUS,
                    bytes,
                    (uint16_t)sizeof(bytes))))
    {
        return false;
    }

    /* STAT按MSB first返回，本层只拼接原始寄存器，不解释状态位。 */
    *status = (uint16_t)(((uint16_t)bytes[0] << 8U) |
                         (uint16_t)bytes[1]);
    return true;
}

static bool gp22_read_result(uint32_t *raw)
{
    uint8_t bytes[4];

    if ((raw == NULL) ||
        (!gp22_read(GP22_CMD_READ_RES0,
                    bytes,
                    (uint16_t)sizeof(bytes))))
    {
        return false;
    }

    /* RES0是有符号16.16格式；符号解释与长度换算留给ALOG。 */
    *raw = ((uint32_t)bytes[0] << 24U) |
           ((uint32_t)bytes[1] << 16U) |
           ((uint32_t)bytes[2] << 8U) |
           (uint32_t)bytes[3];
    return true;
}

static void gp22_reset(void)
{
    /* 复位期间SSN必须保持高；RSTN低1 ms、释放后等待5 ms。 */
    gp22_set_ssn(GPIO_PIN_SET);
    HAL_GPIO_WritePin(GP22_RSTN_GPIO_Port,
                      GP22_RSTN_Pin,
                      GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GP22_RSTN_GPIO_Port,
                      GP22_RSTN_Pin,
                      GPIO_PIN_SET);
    HAL_Delay(5U);
}

static bool length_start_one_pulse(void)
{
    /* TIM1参数完全来自CubeMX；这里只清旧现场并启动一次OPM脉冲。 */
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
    return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK;
}

bool bsp_length_init(void)
{
    uint8_t register_test = 0U;
    uint8_t index;

    g_gp22_initialized = false;
    g_gp22_irq_seen = false;

    if ((hspi1.Instance != SPI1) || (htim1.Instance != TIM1))
    {
        return false;
    }

    gp22_reset();
    if (!gp22_command(GP22_CMD_POWER_ON_RESET))
    {
        return false;
    }

    for (index = 0U; index < 7U; index++)
    {
        if (!gp22_write_config_word(gp22_config_words[index]))
        {
            return false;
        }
    }

    /* B5应读回Reg1最高字节0x01，同时检查命令、SCK和MISO链路。 */
    if ((!gp22_read(GP22_CMD_READ_REG1_MSB, &register_test, 1U)) ||
        (register_test != 0x01U))
    {
        return false;
    }

    g_gp22_initialized = true;
    return true;
}

bool bsp_length_measure(uint32_t *raw,
                        uint16_t *status,
                        uint32_t timeout_ms)
{
    uint32_t start_ms;
    uint32_t measured_raw;
    uint16_t measured_status;
    bool success = false;

    if ((raw == NULL) || (status == NULL) || (timeout_ms == 0U) ||
        (!g_gp22_initialized))
    {
        return false;
    }

    g_gp22_irq_seen = false;
    __HAL_GPIO_EXTI_CLEAR_IT(GP22_INTN_Pin);
    /* INIT清除GP22上一次测量现场，随后才允许产生新的START脉冲。 */
    if (!gp22_command(GP22_CMD_INIT))
    {
        g_gp22_initialized = false;
        return false;
    }

    if (!length_start_one_pulse())
    {
        return false;
    }

    start_ms = HAL_GetTick();
    while ((!g_gp22_irq_seen) &&
           ((HAL_GetTick() - start_ms) < timeout_ms))
    {
        /*
         * EXTI只发布下降沿标志。STA与SP1共节点是否误捕获发射边沿，
         * 只能通过10 m/20 m raw趋势判断，软件不能在这里补偿。
         */
    }

    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    if (g_gp22_irq_seen &&
        gp22_read_status(&measured_status) &&
        gp22_read_result(&measured_raw))
    {
        *raw = measured_raw;
        *status = measured_status;
        success = true;
    }

    g_gp22_irq_seen = false;
    return success;
}

void bsp_length_on_gp22_irq(uint16_t gpio_pin)
{
    if (gpio_pin == GP22_INTN_Pin)
    {
        /* ISR禁止SPI、日志和浮点运算，只通知主循环读取结果。 */
        g_gp22_irq_seen = true;
    }
}
