#ifndef BSP_PATH_H
#define BSP_PATH_H

#include <stdbool.h>

typedef enum
{
    BSP_PATH_NONE = 0,
    BSP_PATH_LENGTH,
    BSP_PATH_RESISTANCE,
    BSP_PATH_CAPACITANCE
} bsp_path_t;

/** @brief 初始化为三路继电器全部释放。 */
void bsp_path_init(void);

/** @brief 先全断后只吸合目标通路；非法参数保持全断并返回false。 */
bool bsp_path_select(bsp_path_t path);

/** @brief 立即释放长度、电阻和电容三路继电器。 */
void bsp_path_off(void);

#endif /* BSP_PATH_H */
