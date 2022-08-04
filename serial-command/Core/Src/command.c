#include "command.h"

char ret[128];
double constants[9];

const char add_command[] = "ADD";
const char lst_command[] = "LST";
const char sel_command[] = "SEL";

const char waiting_for_line[] = "Waiting for line..\r\n";
const char line_error[] = "Could not receive line, timeout or error happened\r\n";
const char command_error[] = "Command error\r\n";

char prompt[] = "Set Servo X Y: ";

/** @brief Receive a single line through serial interface
 * @param huart Pointer to serial interface
 * @param timeout Serial input timeout
 * @retval String containing line or NULL if error
 */
char* command_get_line(UART_HandleTypeDef *huart, uint32_t timeout) {
    //HAL_UART_Transmit(huart, waiting_for_line, sizeof(waiting_for_line), timeout);
    HAL_StatusTypeDef stat;
    char * next_char = ret;
    unsigned char byte = '\0';
    unsigned char* received_byte = &byte;

    while ( *received_byte != '\r' ) {
        stat = HAL_UART_Receive(huart, received_byte, 1U, timeout);

        if ( (stat == HAL_ERROR) || (stat == HAL_TIMEOUT) ) {
            //HAL_UART_Transmit(huart, line_error, sizeof(line_error), timeout);
            return NULL;
        }

        *next_char = *received_byte;
        next_char++;
    } 

    *next_char = '\0';

    return ret;
}

CommandType parse_command(char * str) {
    if ( strncmp(str, add_command, 3) == 0 ) {
        return ADD_COMMAND;
    }
    else if ( strncmp(str, lst_command, 3) == 0) {
        return LST_COMMAND;
    }
    else if ( strncmp(str, sel_command, 3) == 0) {
        return SEL_COMMAND;
    }
}

void print_command_error(UART_HandleTypeDef *huart) {
    HAL_UART_Transmit(huart, command_error, sizeof(command_error), 5000);
}

/**
 * @brief Get PID values through UART interface
 * @param huart pointer to serial interface struct 
 * @retval Pointer to array of PID constants of length 9 or NULL if error
 */ 
double* get_constants(UART_HandleTypeDef *huart) {
    char* line = NULL;
    const char* servo_names = "XYE";
    const char* pid_names = "PID";

    for (int i = 0; i < 3; i++) {
        prompt[10] = servo_names[i];
        for (int j = 0; j < 3; j++) {
            prompt[12] = pid_names[j];
            HAL_UART_Transmit(huart, prompt, 16, 5000);
            line = command_get_line(huart, 1000000);
            constants[i * 3 + j] = atof(line);
        } 
    }
    return constants;
    
    print_command_error(huart);
    return NULL;

}