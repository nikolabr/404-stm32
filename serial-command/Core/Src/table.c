#include "table.h"

TableRow read_row(uint32_t offset) {
    TableRow row = {0};

    uint8_t * ptr = (uint8_t *)TABLE_START_ADDR + offset;

    memcpy(row.header, ptr, 4);
    ptr += 4;

    row.speed = *(uint32_t*)ptr;
    ptr += 4;

    memcpy(row.header, ptr, sizeof(double) * 9);

    return row;  
}

HAL_StatusTypeDef write_row(uint32_t offset, double* constants) {
    TableRow row = {0};

    memcpy(row.header, "ROW", 3);
    row.speed = 0;
    memcpy(row.constants, constants, sizeof(double) * 9);

    return write_words((uint32_t *)&row, offset, sizeof(TableRow) / 4);
}

size_t get_table_size() {
    size_t size = 0;

    uint32_t header = *(uint32_t *)(TABLE_START_ADDR) & 0xFFFFFF;
    while ( header == ROW_HEADER ) {
        size++;
        header = *(uint32_t *)(TABLE_START_ADDR + size * sizeof(TableRow)) & 0xFFFFFF;
    }
    return size;
}

Table get_table() {
    Table table;
    table.size = get_table_size();
    table.rows = (TableRow *) TABLE_START_ADDR;

    return table;
}