#ifndef FLASH_H
#define FLASH_H

#include "stdint.h"
#include "main.h"
#include "table.h"

// Initialize flash interface
HAL_StatusTypeDef flash_init();

// 16-bit and 32-bit: NOTE - THESE ARE ATOMIC OPERATIONS
HAL_StatusTypeDef write_halfword(uint16_t hword, uint32_t offset);
HAL_StatusTypeDef write_word(uint32_t word, uint32_t offset);
HAL_StatusTypeDef write_words(uint32_t * words, uint32_t offset, size_t n);

// 64-bit: not atomic
HAL_StatusTypeDef write_dword(uint64_t dword, uint32_t offset);

#endif