#ifndef APP_PROCESS_H
#define APP_PROCESS_H

void HMI_Process_Init(void);
void HMI_Send_Cmd(const char *cmd_string);
void Debug_printf(const char *text, ...);
void App_Main_Process_Poll(void);

#endif /* APP_PROCESS_H */
