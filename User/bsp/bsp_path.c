#include "bsp_path.h"

#include "main.h"

/* 继电器释放和触点稳定各预留10 ms，任务总时限已包含这部分时间。 */
#define BSP_PATH_SETTLE_DELAY_MS (10U)

void bsp_path_off(void)
{
    /* 三路均为高电平吸合；统一拉低是所有成功/失败流程的安全终态。 */
    HAL_GPIO_WritePin(PATH_LENGTH_EN_GPIO_Port,
                      PATH_LENGTH_EN_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PATH_RESISTANCE_EN_GPIO_Port,
                      PATH_RESISTANCE_EN_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PATH_CAPACITANCE_EN_GPIO_Port,
                      PATH_CAPACITANCE_EN_Pin,
                      GPIO_PIN_RESET);
}

void bsp_path_init(void)
{
    bsp_path_off();
}

bool bsp_path_select(bsp_path_t path)
{
    GPIO_TypeDef *port = NULL;
    uint16_t pin = 0U;

    /*
     * Break-before-make：先释放全部触点再吸合目标继电器，避免长度脉冲、
     * ADC分压和NE555定时网络在切换瞬间互相连接。
     */
    bsp_path_off();
    HAL_Delay(BSP_PATH_SETTLE_DELAY_MS);

    switch (path)
    {
    case BSP_PATH_LENGTH:
        port = PATH_LENGTH_EN_GPIO_Port;
        pin = PATH_LENGTH_EN_Pin;
        break;

    case BSP_PATH_RESISTANCE:
        port = PATH_RESISTANCE_EN_GPIO_Port;
        pin = PATH_RESISTANCE_EN_Pin;
        break;

    case BSP_PATH_CAPACITANCE:
        port = PATH_CAPACITANCE_EN_GPIO_Port;
        pin = PATH_CAPACITANCE_EN_Pin;
        break;

    case BSP_PATH_NONE:
    default:
        return false;
    }

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_Delay(BSP_PATH_SETTLE_DELAY_MS);
    return true;
}
