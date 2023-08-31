/*
 * flash.h
 *
 *  Created on: 04.09.2021
 *      Author: symon
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "stdbool.h"
#include "global_constants.h"

void W25Q128_erase(uint8_t instruction, uint32_t address);
void W25Q128_erase_full_chip();
uint8_t W25Q128_read_status_register(uint8_t);
void W25Q128_read_unique_ID(uint8_t* memory_address);
uint32_t W25Q128_read_JEDEC_ID();
void W25Q128_write_data(uint32_t memory_address, uint8_t* data, int number_of_bytes);
void W25Q128_fast_write_data(uint32_t memory_address, uint8_t* data, int number_of_bytes);
void W25Q128_unsafe_write_data(uint32_t memory_address, uint8_t* data, int number_of_bytes);
void W25Q128_modify_data(uint32_t memory_address, uint8_t* new_data, int number_of_bytes);
void W25Q128_read_data(uint32_t memory_address, uint8_t* pointer_for_data, int number_of_bytes);
void W25Q128_fast_read_data(uint32_t memory_address, uint8_t* pointer_for_data, int number_of_bytes);
void flash_save(uint8_t);
uint16_t flash_flush();
bool W25Q128_check_if_busy();
uint16_t flash_get_write_buffer_free_space();
uint16_t flash_get_write_buffer_size();

//-------MACROs FOR FLASH W25Q128JV----------

#define W25Q128_PAGE_SIZE 256
#define W25Q128_SECTOR_SIZE 16*W25Q128_PAGE_SIZE
#define W25Q128_SECTOR_COUNT 4096
#define FLASH_WRITE_ENABLE 0x06
#define FLASH_WRITE_DISABLE 0x04

#define FLASH_JEDEC_ID 0x9F
#define FLASH_READ_UNIQUE_ID 0x4B

#define FLASH_READ_DATA 0x03
#define FLASH_FAST_READ 0x0B
#define FLASH_PAGE_PROGRAM 0x02

#define FLASH_ERASE_SECTOR_4KB 0x20
#define FLASH_ERASE_BLOCK_32KB 0x52
#define FLASH_ERASE_BLOCK_64KB 0xD8
#define FLASH_ERASE_CHIP 0x60

#define FLASH_READ_STATUS_REGISTER_1 0x05
#define FLASH_WRITE_STATUS_REGISTER_1 0x01
#define FLASH_READ_STATUS_REGISTER_2 0x35
#define FLASH_WRITE_STATUS_REGISTER_2 0x31
#define FLASH_READ_STATUS_REGISTER_3 0x15
#define FLASH_WRITE_STATUS_REGISTER_3 0x11

#define FLASH_POWER_DOWN 0xB9

#define FLASH_ENABLE_RESET 0x66
#define FLASH_RESET_DEVICE 0x99

#define FLASH_DUMMY_BYTE 0xBD
#define FLASH_IS_BUSY_BIT 0x01

#define FLASH_READ_BLOCK_0 0x000000
#define FLASH_READ_BLOCK_1 0x010000
#define FLASH_READ_BLOCK_2 0x020000
#define FLASH_READ_BLOCK_3 0x030000
#define FLASH_READ_BLOCK_4 0x040000
#define FLASH_READ_BLOCK_5 0x050000
#define FLASH_READ_BLOCK_6 0x060000
#define FLASH_READ_BLOCK_7 0x070000

#endif /* FLASH_H_ */
