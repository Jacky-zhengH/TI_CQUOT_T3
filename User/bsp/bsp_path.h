#ifndef BSP_PATH_H
#define BSP_PATH_H

#include <stdbool.h>

typedef enum
{
    BSP_PATH_NONE = 0,       /* 三个继电器全部断开。 */
    BSP_PATH_LENGTH,         /* 接通TDR/GP22长度测量通路。 */
    BSP_PATH_RESISTANCE,     /* 接通直流分压电阻测量通路。 */
    BSP_PATH_CAPACITANCE     /* 接通NE555电容测量通路。 */
} bsp_path_t;

/** @brief 初始化通路控制并强制断开全部继电器。 */
void bsp_path_init(void);

/**
 * @brief 采用先断后通方式同步选择一路测量通路。
 * @param path 目标通路；传入BSP_PATH_NONE等同于全部关闭。
 * @return 合法通路切换成功返回true，非法枚举返回false并保持全断。
 * @note 非NONE切换固定阻塞约20 ms，只能在主循环上下文调用。
 */
bool bsp_path_select(bsp_path_t path);

/** @brief 立即断开全部测量通路。 */
void bsp_path_off(void);

#endif /* BSP_PATH_H */
