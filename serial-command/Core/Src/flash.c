#include "flash.h"

HAL_StatusTypeDef flash_init() {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef FLASH_EraseInitStruct = {0};

    FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    FLASH_EraseInitStruct.PageAddress = TABLE_START_ADDR;
    FLASH_EraseInitStruct.NbPages = 1;

    uint32_t error = 0;
    return HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &error);
}

HAL_StatusTypeDef write_halfword(uint16_t hword, uint32_t offset) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, TABLE_START_ADDR + offset, hword);
}

HAL_StatusTypeDef write_dword(uint64_t dword, uint32_t offset) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, TABLE_START_ADDR + offset, dword);
}

HAL_StatusTypeDef write_word(uint32_t word, uint32_t offset) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, TABLE_START_ADDR + offset, word);
}

HAL_StatusTypeDef write_words(uint32_t * words, uint32_t offset, size_t n) {
    HAL_StatusTypeDef stat = HAL_OK;
    for (int i = 0; i < n; i++) {
        stat |= write_word(words[i], offset + i * 4);
    }
    return stat;
}