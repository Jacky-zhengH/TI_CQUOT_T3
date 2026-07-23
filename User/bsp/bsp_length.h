#ifndef BSP_LENGTH_H
#define BSP_LENGTH_H

#include <stdint.h>

/*
 * 长度测量原始驱动
 * ----------------
 * 本模块通过硬件SPI配置TDC-GP22，并用TIM1在PE9产生一次外部START脉冲。
 * 成功后返回GP22的RES0原始16.16结果和STAT寄存器，不在BSP内换算时间或长度。
 */

typedef enum
{
    BSP_LENGTH_RESULT_OK = 0,       /* 初始化或单次原始测量正常完成。 */
    BSP_LENGTH_RESULT_INVALID_PARAM,/* 输出指针为空或timeout_ms为0。 */
    BSP_LENGTH_RESULT_NOT_READY,    /* 未初始化、通路不正确或INTN初态异常。 */
    BSP_LENGTH_RESULT_SPI_ERROR,    /* GP22 SPI发送、接收或配置读回失败。 */
    BSP_LENGTH_RESULT_PULSE_ERROR,  /* TIM1不是单脉冲模式或脉冲启动失败。 */
    BSP_LENGTH_RESULT_TIMEOUT,      /* 软件等待或GP22内部TDC发生超时。 */
    BSP_LENGTH_RESULT_NO_STOP_HIT,  /* STAT表明STOP1没有捕获到反射边沿。 */
    BSP_LENGTH_RESULT_BAD_STATUS    /* STAT、ALU指针或RES0内容不可信。 */
} bsp_length_result_t;

/** @brief 复位、配置并验证TDC-GP22。 */
bsp_length_result_t bsp_length_init(void);

/**
 * @brief 在有限超时内完成一次GP22原始16.16测量。
 * @param raw_result 返回GP22 RES0原始有符号16.16值。
 * @param status_register 返回与本次结果对应的16位STAT。
 * @param timeout_ms 等待GP22 INTN有效的最大毫秒数。
 * @return 单次测量结果；仅返回OK时两个输出参数有效。
 * @note 调用前必须先用bsp_path_select()接通长度通路。
 */
bsp_length_result_t bsp_length_measure_raw(int32_t *raw_result,
                                            uint16_t *status_register,
                                            uint32_t timeout_ms);

/**
 * @brief 处理HAL转发的GP22 INTN下降沿。
 * @note ISR中只设置volatile标志，SPI和状态解析全部留在主循环执行。
 */
void bsp_length_on_gp22_irq(uint16_t gpio_pin);

#endif /* BSP_LENGTH_H */
