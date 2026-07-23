#include "bsp_length.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp_board.h"
#include "main.h"
#include "spi.h"
#include "tim.h"

/*
 * 每次SPI HAL调用最多等待2 ms，避免模块断线时长期卡住主循环。
 * RSTN低1 ms远大于GP22最小复位脉宽；释放后再等1 ms让数字部分稳定。
 * 配置Reg0启动高速振荡器后保守等待5 ms，兼顾未知的晶体/陶瓷器件。
 * 1024是TIM1单脉冲结束的CPU轮询上限，不代表1024 ms；正常约1 us即结束。
 */
#define BSP_LENGTH_SPI_TIMEOUT_MS              (2U)
#define BSP_LENGTH_RESET_LOW_DELAY_MS          (1U)
#define BSP_LENGTH_RESET_STABLE_DELAY_MS       (1U)
#define BSP_LENGTH_OSCILLATOR_STABLE_DELAY_MS  (5U)
#define BSP_LENGTH_PULSE_POLL_LIMIT             (1024U)

/* GP22单字节命令，以及写配置/读结果命令的高4位基址。 */
#define GP22_OPCODE_POWER_ON_RESET              (0x50U)
#define GP22_OPCODE_INIT                        (0x70U)
#define GP22_OPCODE_WRITE_CONFIG_BASE           (0x80U)
#define GP22_OPCODE_READ_BASE                   (0xB0U)

/* 读命令低地址：0读RES0，4读STAT，5读Reg1最高字节用于通信校验。 */
#define GP22_READ_RES0                          (0U)
#define GP22_READ_STATUS                        (4U)
#define GP22_READ_REG1_MSB                      (5U)

/*
 * STAT位域：低3位指出最后写入哪个RES寄存器，Bit5..3记录STOP1命中数；
 * Bit9是TDC超时。0x7C00集中覆盖预计数、开短路及EEPROM等异常位。
 */
#define GP22_STATUS_ALU_POINTER_MASK            (0x0007U)
#define GP22_STATUS_STOP1_HITS_MASK             (0x0038U)
#define GP22_STATUS_STOP1_HITS_SHIFT            (3U)
#define GP22_STATUS_TIMEOUT_TDC                 (0x0200U)
#define GP22_STATUS_UNEXPECTED_ERROR_MASK       (0x7C00U)

/*
 * 已按GP22数据手册核对的24位配置载荷：
 * Reg0 Mode1/自动校准/CLKHS持续；Reg1计算STOP1-START；
 * Reg2只开TDC超时与ALU Ready中断；Reg3~6关闭本题未用功能。
 */
static const uint32_t s_gp22_config[7] =
{
    0x029660U, /* Reg0：Mode1、自动校准、CLKHS持续运行。 */
    0x010100U, /* Reg1：ALU计算第一个STOP1减START。 */
    0xA00000U, /* Reg2：使能TDC超时和ALU Ready中断。 */
    0x180000U, /* Reg3：关闭First Wave和自动多结果计算。 */
    0x200100U, /* Reg4：保留默认位并关闭脉宽测量。 */
    0x080000U, /* Reg5：关闭FIRE输出和相移功能。 */
    0x000000U  /* Reg6：使用数字STOP、单倍分辨率。 */
};

/* initialized只在完整配置和B5读回均通过后置位；IRQ标志由EXTI写入。 */
static bool s_initialized;
static volatile bool s_irq_pending;

static void gp22_select(void)
{
    /* SSN低有效；一次完整SPI事务期间保持为低。 */
    HAL_GPIO_WritePin(GP22_SSN_GPIO_Port,
                      GP22_SSN_Pin,
                      GPIO_PIN_RESET);
}

static void gp22_deselect(void)
{
    /* HAL GPIO调用开销已明显大于GP22要求的50 ns片选高电平时间。 */
    HAL_GPIO_WritePin(GP22_SSN_GPIO_Port,
                      GP22_SSN_Pin,
                      GPIO_PIN_SET);
}

static bool gp22_send(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }
    /* 无论HAL发送成功还是失败，退出前都恢复SSN为高，避免锁住总线。 */
    gp22_select();
    status = HAL_SPI_Transmit(&hspi1,
                              (uint8_t *)data,
                              length,
                              BSP_LENGTH_SPI_TIMEOUT_MS);
    gp22_deselect();
    return status == HAL_OK;
}

static bool gp22_send_opcode(uint8_t opcode)
{
    return gp22_send(&opcode, 1U);
}

static bool gp22_read(uint8_t address, uint8_t *data, uint16_t length)
{
    uint8_t opcode = (uint8_t)(GP22_OPCODE_READ_BASE | address);
    /* SPI读取也必须发送时钟，dummy字节仅用于产生SCK，不参与协议内容。 */
    uint8_t dummy[4] = {0U, 0U, 0U, 0U};
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0U) ||
        (length > (uint16_t)sizeof(dummy)))
    {
        return false;
    }

    /* 先发送B0|地址，再在同一次SSN低电平窗口中读取1~4字节。 */
    gp22_select();
    status = HAL_SPI_Transmit(&hspi1,
                              &opcode,
                              1U,
                              BSP_LENGTH_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        status = HAL_SPI_TransmitReceive(&hspi1,
                                         dummy,
                                         data,
                                         length,
                                         BSP_LENGTH_SPI_TIMEOUT_MS);
    }
    gp22_deselect();
    return status == HAL_OK;
}

static bool gp22_write_config(uint8_t address, uint32_t value)
{
    uint8_t transaction[4];

    if ((address >= 7U) || ((value & 0xFF000000UL) != 0U))
    {
        return false;
    }
    /* GP22配置写事务固定为“80|地址 + 24位载荷”，高字节先发送。 */
    transaction[0] = (uint8_t)(GP22_OPCODE_WRITE_CONFIG_BASE | address);
    transaction[1] = (uint8_t)(value >> 16U);
    transaction[2] = (uint8_t)(value >> 8U);
    transaction[3] = (uint8_t)value;
    return gp22_send(transaction, (uint16_t)sizeof(transaction));
}

static bool gp22_read_status(uint16_t *status_register)
{
    uint8_t data[2];

    if ((status_register == NULL) ||
        (!gp22_read(GP22_READ_STATUS, data, (uint16_t)sizeof(data))))
    {
        return false;
    }
    /* GP22按MSB First返回，两个字节重新组合为主机端uint16_t。 */
    *status_register = (uint16_t)(((uint16_t)data[0] << 8U) |
                                  (uint16_t)data[1]);
    return true;
}

static bool gp22_read_result(int32_t *raw_result)
{
    uint8_t data[4];
    uint32_t value;

    if ((raw_result == NULL) ||
        (!gp22_read(GP22_READ_RES0, data, (uint16_t)sizeof(data))))
    {
        return false;
    }
    /* RES0是32位有符号16.16原始值；这里只拼接，不做时间/长度换算。 */
    value = ((uint32_t)data[0] << 24U) |
            ((uint32_t)data[1] << 16U) |
            ((uint32_t)data[2] << 8U) |
            (uint32_t)data[3];
    *raw_result = (int32_t)value;
    return true;
}

static bool gp22_hardware_reset(void)
{
    /* 复位期间保持SSN不选中，避免RSTN变化被误解释成SPI事务。 */
    gp22_deselect();
    HAL_GPIO_WritePin(GP22_RSTN_GPIO_Port,
                      GP22_RSTN_Pin,
                      GPIO_PIN_RESET);
    HAL_Delay(BSP_LENGTH_RESET_LOW_DELAY_MS);
    HAL_GPIO_WritePin(GP22_RSTN_GPIO_Port,
                      GP22_RSTN_Pin,
                      GPIO_PIN_SET);
    HAL_Delay(BSP_LENGTH_RESET_STABLE_DELAY_MS);
    return HAL_GPIO_ReadPin(GP22_RSTN_GPIO_Port,
                            GP22_RSTN_Pin) == GPIO_PIN_SET;
}

static bool bsp_length_path_selected(void)
{
    /* 长度继电器必须独占：长度ON，同时电阻和电容两路均为OFF。 */
    return (HAL_GPIO_ReadPin(PATH_LENGTH_EN_GPIO_Port,
                             PATH_LENGTH_EN_Pin) == GPIO_PIN_SET) &&
           (HAL_GPIO_ReadPin(PATH_RESISTANCE_EN_GPIO_Port,
                             PATH_RESISTANCE_EN_Pin) == GPIO_PIN_RESET) &&
           (HAL_GPIO_ReadPin(PATH_CAPACITANCE_EN_GPIO_Port,
                             PATH_CAPACITANCE_EN_Pin) == GPIO_PIN_RESET);
}

static void bsp_length_stop_pulse(void)
{
    /* 停止后同时清计数器和更新标志，为下一次单脉冲建立相同起点。 */
    if (htim1.Instance == TIM1)
    {
        (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        __HAL_TIM_SET_COUNTER(&htim1, 0U);
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    }
}

static bool bsp_length_trigger_pulse(void)
{
    uint32_t poll_count = 0U;

    bsp_length_stop_pulse();
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    /* OPM应在约1 us后自动清CEN；轮询上限防止配置异常时死循环。 */
    while (((htim1.Instance->CR1 & TIM_CR1_CEN) != 0U) &&
           (poll_count < BSP_LENGTH_PULSE_POLL_LIMIT))
    {
        poll_count++;
    }
    if ((htim1.Instance->CR1 & TIM_CR1_CEN) != 0U)
    {
        bsp_length_stop_pulse();
        return false;
    }

    bsp_length_stop_pulse();
    return true;
}

bsp_length_result_t bsp_length_init(void)
{
    uint8_t register_index;
    uint8_t config1_msb;

    /* 初始化流程：安全电平 -> 外设检查 -> 硬复位/POR -> 配置 -> B5校验。 */
    s_initialized = false;
    s_irq_pending = false;
    gp22_deselect();
    HAL_GPIO_WritePin(GP22_RSTN_GPIO_Port,
                      GP22_RSTN_Pin,
                      GPIO_PIN_SET);

    if ((hspi1.Instance != SPI1) || (htim1.Instance != TIM1))
    {
        return BSP_LENGTH_RESULT_NOT_READY;
    }
    if ((htim1.Instance->CR1 & TIM_CR1_OPM) == 0U)
    {
        return BSP_LENGTH_RESULT_PULSE_ERROR;
    }
    if ((!gp22_hardware_reset()) ||
        (!gp22_send_opcode(GP22_OPCODE_POWER_ON_RESET)))
    {
        return BSP_LENGTH_RESULT_SPI_ERROR;
    }

    /* 地址0~6与配置表索引一一对应，任何一次写失败都立即终止。 */
    for (register_index = 0U; register_index < 7U; register_index++)
    {
        if (!gp22_write_config(register_index,
                               s_gp22_config[register_index]))
        {
            return BSP_LENGTH_RESULT_SPI_ERROR;
        }
    }
    HAL_Delay(BSP_LENGTH_OSCILLATOR_STABLE_DELAY_MS);

    /* Reg1最高字节应为0x01；它同时验证命令、MISO和已写配置是否正常。 */
    if (!gp22_read(GP22_READ_REG1_MSB, &config1_msb, 1U))
    {
        return BSP_LENGTH_RESULT_SPI_ERROR;
    }
    if (config1_msb != 0x01U)
    {
        return BSP_LENGTH_RESULT_BAD_STATUS;
    }
    if (!gp22_send_opcode(GP22_OPCODE_INIT))
    {
        return BSP_LENGTH_RESULT_SPI_ERROR;
    }
    if (HAL_GPIO_ReadPin(GP22_INTN_GPIO_Port,
                         GP22_INTN_Pin) != GPIO_PIN_SET)
    {
        return BSP_LENGTH_RESULT_NOT_READY;
    }

    s_initialized = true;
    return BSP_LENGTH_RESULT_OK;
}

bsp_length_result_t bsp_length_measure_raw(int32_t *raw_result,
                                            uint16_t *status_register,
                                            uint32_t timeout_ms)
{
    uint32_t start_ms;
    uint32_t stop1_hits;
    uint32_t alu_pointer;
    int32_t measured_raw;
    uint16_t measured_status;

    if ((raw_result == NULL) || (status_register == NULL) ||
        (timeout_ms == 0U))
    {
        return BSP_LENGTH_RESULT_INVALID_PARAM;
    }
    if ((!s_initialized) || (!bsp_length_path_selected()))
    {
        return BSP_LENGTH_RESULT_NOT_READY;
    }

    /*
     * 单次流程：INIT清现场 -> 确认INTN高 -> 发START脉冲 -> 等INTN ->
     * 读STAT并筛错 -> 读RES0 -> 再次INIT恢复GP22待命状态。
     */
    s_irq_pending = false;
    if (!gp22_send_opcode(GP22_OPCODE_INIT))
    {
        s_initialized = false;
        return BSP_LENGTH_RESULT_SPI_ERROR;
    }
    /* INIT之后INTN应为高；若仍为低，旧中断或硬件故障会污染本次结果。 */
    if (HAL_GPIO_ReadPin(GP22_INTN_GPIO_Port,
                         GP22_INTN_Pin) != GPIO_PIN_SET)
    {
        return BSP_LENGTH_RESULT_NOT_READY;
    }
    if (!bsp_length_trigger_pulse())
    {
        (void)gp22_send_opcode(GP22_OPCODE_INIT);
        return BSP_LENGTH_RESULT_PULSE_ERROR;
    }

    start_ms = bsp_now_ms();
    while ((!s_irq_pending) &&
           (HAL_GPIO_ReadPin(GP22_INTN_GPIO_Port,
                             GP22_INTN_Pin) == GPIO_PIN_SET) &&
           (!bsp_timeout(start_ms, timeout_ms)))
    {
        /* EXTI只置标志；SPI和状态检查留在主循环上下文。 */
    }
    s_irq_pending = false;

    if (HAL_GPIO_ReadPin(GP22_INTN_GPIO_Port,
                         GP22_INTN_Pin) == GPIO_PIN_SET)
    {
        (void)gp22_send_opcode(GP22_OPCODE_INIT);
        return BSP_LENGTH_RESULT_TIMEOUT;
    }
    if (!gp22_read_status(&measured_status))
    {
        s_initialized = false;
        return BSP_LENGTH_RESULT_SPI_ERROR;
    }
    /* 先处理硬件超时和集中异常位，再检查STOP1命中数及ALU结果指针。 */
    if ((measured_status & GP22_STATUS_TIMEOUT_TDC) != 0U)
    {
        (void)gp22_send_opcode(GP22_OPCODE_INIT);
        return BSP_LENGTH_RESULT_TIMEOUT;
    }
    if ((measured_status & GP22_STATUS_UNEXPECTED_ERROR_MASK) != 0U)
    {
        (void)gp22_send_opcode(GP22_OPCODE_INIT);
        return BSP_LENGTH_RESULT_BAD_STATUS;
    }

    stop1_hits = ((uint32_t)measured_status &
                  GP22_STATUS_STOP1_HITS_MASK) >>
                 GP22_STATUS_STOP1_HITS_SHIFT;
    if (stop1_hits == 0U)
    {
        (void)gp22_send_opcode(GP22_OPCODE_INIT);
        return BSP_LENGTH_RESULT_NO_STOP_HIT;
    }
    alu_pointer = (uint32_t)measured_status &
                  GP22_STATUS_ALU_POINTER_MASK;
    if (alu_pointer != 1U)
    {
        (void)gp22_send_opcode(GP22_OPCODE_INIT);
        return BSP_LENGTH_RESULT_BAD_STATUS;
    }

    /* 全1通常表示无效/未更新结果，不允许作为真实飞行时间交给ALOG。 */
    if ((!gp22_read_result(&measured_raw)) ||
        ((uint32_t)measured_raw == 0xFFFFFFFFUL))
    {
        (void)gp22_send_opcode(GP22_OPCODE_INIT);
        return BSP_LENGTH_RESULT_BAD_STATUS;
    }
    if (!gp22_send_opcode(GP22_OPCODE_INIT))
    {
        s_initialized = false;
        return BSP_LENGTH_RESULT_SPI_ERROR;
    }

    *raw_result = measured_raw;
    *status_register = measured_status;
    return BSP_LENGTH_RESULT_OK;
}

void bsp_length_on_gp22_irq(uint16_t gpio_pin)
{
    /* 统一EXTI回调可能转发其他引脚，仅接收GP22 INTN。 */
    if (gpio_pin == GP22_INTN_Pin)
    {
        s_irq_pending = true;
    }
}
