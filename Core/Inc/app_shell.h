#ifndef APP_SHELL_H
#define APP_SHELL_H

/* Initialise USART2 (115200 8N1) and enable interrupt-driven reception. */
void Shell_Init(void);

/* FreeRTOS task entry: line editing and command execution. */
void Shell_Task(void *argument);

#endif /* APP_SHELL_H */
