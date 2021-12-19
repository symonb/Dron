/*
 * flash.h
 *
 *  Created on: 04.09.2021
 *      Author: symon
 */

#ifndef FLASH_H_
#define FLASH_H_

void setup_FLASH();
void flash_SPI_read(uint8_t, uint8_t *, uint8_t);
void flash_SPI_write(uint8_t , uint8_t * , uint8_t);
void flash_SPI_write_DMA(uint8_t , uint8_t * , uint8_t);
void flash_erase(uint8_t instruction, uint8_t *address);
void flash_full_chip_erase();
uint8_t flash_read_status_register(uint8_t );
void flash_save_data(uint8_t ,uint32_t , uint8_t *, uint8_t);
void flash_read_data(uint8_t ,uint32_t , uint8_t *, uint8_t);

//-------MACRO FOR FLASH W25Q128JV----------
#define FLASH_WRITE_ENABLE 0x06
#define FLASH_WRITE_DISABLE 0x04

#define FLASH_JEDEC_ID 0x9F
#define FLASH_READ_UNIQUE_ID 0x4B

#define FLASH_READ_DATA 0x03
#define FLASH_PAGE_PROGRAM 0x02

#define FLASH_SECTOR_ERASE_4KB 0x20
#define FLASH_BLOCK_ERASE_32KB 0x52
#define FLASH_BLOCK_ERASE_64KB 0xD8
#define FLASH_CHIP_ERASE 0x60

#define FLASH_READ_STATUS_REGISTER_1 0x05
#define FLASH_WRITE_STATUS_REGISTER_1 0x01
#define FLASH_READ_STATUS_REGISTER_2 0x35
#define FLASH_WRITE_STATUS_REGISTER_2 0x31
#define FLASH_READ_STATUS_REGISTER_3 0x15
#define FLASH_WRITE_STATUS_REGISTER_3 0x11

#define FLASH_POWER_DOWN 0xB9

#define FLASH_ENABLE_RESET 0x66
#define FLASH_RESET_DEVICE 0x99

#define FLASH_DUMMY_BYTE 0xF0

#define FLASH_READ_BLOCK_0 0x000000
#define FLASH_READ_BLOCK_1 0x010000
#define FLASH_READ_BLOCK_2 0x020000
#define FLASH_READ_BLOCK_3 0x030000
#define FLASH_READ_BLOCK_4 0x040000
#define FLASH_READ_BLOCK_5 0x050000
#define FLASH_READ_BLOCK_6 0x060000
#define FLASH_READ_BLOCK_7 0x070000




#endif /* FLASH_H_ */
