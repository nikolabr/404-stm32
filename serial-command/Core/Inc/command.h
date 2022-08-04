#ifndef COMMAND_H
#define COMMAND_H

#include "main.h"
#include "stdlib.h"

double* get_constants(UART_HandleTypeDef *huart);
char* command_get_line(UART_HandleTypeDef *huart, uint32_t timeout);

typedef enum {
    ADD_COMMAND,
    LST_COMMAND,
    SEL_COMMAND
} CommandType;

CommandType parse_command(char * str);

#endif