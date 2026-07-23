#ifndef __APP_PROCESS_H
#define __APP_PROCESS_H

#include "bsp_flash.h"

//=======================================
// 函数声明
//=======================================
void HMI_Process_Init(void);
void HMI_Send_Cmd(const char *cmd_string);
void Debug_printf(const char *text, ...);
void App_Main_Process_Poll(void);

/**
 * @brief 在APP确认测量停止且继电器关闭后，显式保存RAM标定副本。
 * @note 仅供后续标定/HMI流程在主循环上下文调用，不会在普通测量后自动写Flash。
 */
bsp_flash_result_t App_Calibration_Save(void);

#endif
