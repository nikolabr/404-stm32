#ifndef TABLE_H
#define TABLE_H

/** 
 * Address where the coefficient table starts.
 * On the Nucleo F303K8, this is the second last page of memory.
 * For other valid memory pages, refer to the datasheet, but do note that the MCU itself supports more memory than is actually on the said board.
 * */ 
#define TABLE_START_ADDR 0x0800F000

// 32-bit number representation of the row header
#define ROW_HEADER 0x00574f52

#include "flash.h"
#include "stdlib.h"
#include "string.h"

typedef struct {
    char header[4];
    uint32_t speed;
    double constants[9];
} TableRow;

typedef struct {
    size_t size;
    TableRow* rows;
} Table;

TableRow read_row(uint32_t offset);
HAL_StatusTypeDef write_row(uint32_t offset, double* constants);

Table get_table();

#endif