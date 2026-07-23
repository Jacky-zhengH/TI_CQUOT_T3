#include "bsp_path.h"

#include "main.h"

/*
 * 继电器机械动作远慢于GPIO电平变化：
 * RELEASE用于等待旧触点完全释放，SETTLE用于等待新触点吸合稳定。
 * 两个10 ms是当前保守初值，最终应以继电器数据手册和示波器实测为准。
 */
#define BSP_PATH_RELEASE_DELAY_MS    (10U)
#define BSP_PATH_SETTLE_DELAY_MS     (10U)

/* 当前驱动为高电平吸合、低电平断开。 */
#define BSP_PATH_RELAY_ON            GPIO_PIN_SET
#define BSP_PATH_RELAY_OFF           GPIO_PIN_RESET

static void bsp_path_all_off(void)
{
    /* 所有安全退出都汇聚到本函数，避免漏关某一路继电器。 */
    HAL_GPIO_WritePin(PATH_LENGTH_EN_GPIO_Port,
                      PATH_LENGTH_EN_Pin,
                      BSP_PATH_RELAY_OFF);
    HAL_GPIO_WritePin(PATH_RESISTANCE_EN_GPIO_Port,
                      PATH_RESISTANCE_EN_Pin,
                      BSP_PATH_RELAY_OFF);
    HAL_GPIO_WritePin(PATH_CAPACITANCE_EN_GPIO_Port,
                      PATH_CAPACITANCE_EN_Pin,
                      BSP_PATH_RELAY_OFF);
}

/*
 * 接通目标前再次执行全断，是第二道互锁保护。
 * 因此即使未来调用顺序被修改，也不会从这里同时打开两条通路。
 */
static bool bsp_path_enable_one(bsp_path_t path)
{
    bsp_path_all_off();

    switch (path)
    {
    case BSP_PATH_LENGTH:
        HAL_GPIO_WritePin(PATH_LENGTH_EN_GPIO_Port,
                          PATH_LENGTH_EN_Pin,
                          BSP_PATH_RELAY_ON);
        return true;

    case BSP_PATH_RESISTANCE:
        HAL_GPIO_WritePin(PATH_RESISTANCE_EN_GPIO_Port,
                          PATH_RESISTANCE_EN_Pin,
                          BSP_PATH_RELAY_ON);
        return true;

    case BSP_PATH_CAPACITANCE:
        HAL_GPIO_WritePin(PATH_CAPACITANCE_EN_GPIO_Port,
                          PATH_CAPACITANCE_EN_Pin,
                          BSP_PATH_RELAY_ON);
        return true;

    default:
        return false;
    }
}

void bsp_path_init(void)
{
    /* MX_GPIO_Init已经配置GPIO模式；BSP初始化只负责恢复安全输出状态。 */
    bsp_path_all_off();
}

bool bsp_path_select(bsp_path_t path)
{
    /* Break-Before-Make第一步：无条件断开旧通路。 */
    bsp_path_all_off();

    if (path == BSP_PATH_NONE)
    {
        /* NONE不需要等待机械吸合，保持全断即可返回。 */
        return true;
    }
    if ((path != BSP_PATH_LENGTH) &&
        (path != BSP_PATH_RESISTANCE) &&
        (path != BSP_PATH_CAPACITANCE))
    {
        return false;
    }

    /* 等旧触点释放后，只接通经过枚举校验的一路。 */
    HAL_Delay(BSP_PATH_RELEASE_DELAY_MS);
    if (!bsp_path_enable_one(path))
    {
        bsp_path_all_off();
        return false;
    }
    /* 新触点稳定后才允许APP继续启动ADC、NE555或GP22测量。 */
    HAL_Delay(BSP_PATH_SETTLE_DELAY_MS);
    return true;
}

void bsp_path_off(void)
{
    bsp_path_all_off();
}
