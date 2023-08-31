/*
 * OSD.h
 *
 *  Created on: 04.09.2021
 *      Author: symon
 */

#ifndef OSD_H_
#define OSD_H_

 //-------MACROS FOR OSD MAX7456-------------
 //-------ADDRESSES OF REGISTERS-------------
#define OSD_VM0 0x00   // Video Mode 0
#define OSD_VM1 0x01   // Video Mode 1
#define OSD_HOS 0x02   // Horizontal Offset
#define OSD_VOS 0x03   // Vertical Offset
#define OSD_DMM 0x04   // Display Memory Mode
#define OSD_DMAH 0x05  // Display Memory Address High
#define OSD_DMAL 0x06  // Display Memory Address Low
#define OSD_DMDI 0x07  // Display Memory Data In
#define OSD_CMM 0x08   // Character Memory Mode
#define OSD_CMAH 0x09  // Character Memory Address High
#define OSD_CMAL 0x0A  // Character Memory Address Low
#define OSD_CMDI 0x0B  // Character Memory Data In
#define OSD_OSDM 0x0C  // OSD Insertion Mux
#define OSD_RB0 0x10   // Row 0 Brightness
#define OSD_RB1 0x11   // Row 1 Brightness
#define OSD_RB2 0x12   // Row 2 Brightness
#define OSD_RB3 0x13   // Row 3 Brightness
#define OSD_RB4 0x14   // Row 4 Brightness
#define OSD_RB5 0x15   // Row 5 Brightness
#define OSD_RB6 0x16   // Row 6 Brightness
#define OSD_RB7 0x17   // Row 7 Brightness
#define OSD_RB8 0x18   // Row 8 Brightness
#define OSD_RB9 0x19   // Row 9 Brightness
#define OSD_RB10 0x1A  // Row 10 Brightness
#define OSD_RB11 0x1B  // Row 11 Brightness
#define OSD_RB12 0x1C  // Row 12 Brightness
#define OSD_RB13 0x1D  // Row 13 Brightness
#define OSD_RB14 0x1E  // Row 14 Brightness
#define OSD_RB15 0x1F  // Row 15 Brightness
#define OSD_OSDBL 0x6C // OSD Black Level
//----------ONLY FOR READING-----------
#define OSD_STAT 0x20 // Status    0x20 = 0xA0-0x80    (0x80 will be added in receiving function)
#define OSD_DMDO 0x30 // Display Memory Data Out   0x30 = 0xB0-0x80    (0x80 will be added in receiving function)
#define OSD_CMDO 0x40 // Character Memory Data Out     0x40 = 0xC0-0x80    (0x80 will be added in receiving function)

//----------IMPORTANT FLAGS and commands------------
#define OSD_BUSY_FLAG 0x20 // it is set when Character Memory is unavailable to be written or read from
#define OSD_CLEAR_DM 0x04  //  is set when Display Memory is clearing and is reset when is done
#define OSD_NTSC_SIGNAL_DETECTED 0x02
#define OSD_PAL_SIGNAL_DETECTED 0x01
#define OSD_TERMINATE_AUTO_INCREMENT 0xFF

#define OSD_MAX_CHARACTER_NUMBER 256

bool OSD_init();
void OSD_SPI_write(uint8_t instruction, uint8_t data);
void OSD_SPI_read(uint8_t instruction, uint8_t* memory_address);
void OSD_write_new_character(const uint8_t* new_character_table, uint8_t character_number);
void OSD_read_character(uint8_t* table_to_save_character, uint8_t character_number);
void OSD_clear_Display_Memory();

void OSD_write_to_Display_Memory_8bit(uint8_t character_number, uint16_t character_position_on_display, uint8_t attributes);

void OSD_write_to_Display_Memory_16bit(uint8_t character_number, uint16_t character_position_on_display);
void OSD_write_to_Display_Memory_16bit_AI(uint8_t* character_number_tab, uint16_t first_character_position_on_display, uint16_t string_length);
bool is_OSD_busy();
void OSD_print_logo();
void OSD_print_battery_voltage();
void OSD_print_battery_cell_voltage();
void OSD_print_time();
void OSD_print_flight_mode();
void OSD_print_warnings();
void OSD_update_logo_characters();

typedef struct
{
    char* chip_name;
    bool calibrated;
    timeUs_t logo_time;
} osd_t;

extern osd_t main_OSD;

#endif /* OSD_H_ */
