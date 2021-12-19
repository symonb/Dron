/**
  ******************************************************************************
    * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/



#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "setup.h"
#include "stabilize.h"
#include "acro.h"
#include "ibus.h"
#include "connection.h"
#include "MPU6000.h"
#include "flash.h"

//for debugging only:
static int pik = 0;
static int pek = 0;
uint8_t test1;
uint8_t test2;
int main(void) {

	setup();
	turn_OFF_BLUE_LED();
	setup_NVIC_1();
	delay_mili(1000);
	setup_MPU6000();
	setup_NVIC_2();

	static float time_flag0_1 = 0;
	static float time_flag0_2 = 0;
	static float time_flag0_3 = 0;

	//for flash debugging:


	//first 3 bytes have to be address of memory to save data:
	flash_write_buffer[0]=(FLASH_READ_BLOCK_0>>16)&0xFF;
	flash_write_buffer[1]=(FLASH_READ_BLOCK_0>>8)&0xFF;
	flash_write_buffer[2]=(FLASH_READ_BLOCK_0)&0xFF;
	for(int i=3;i<20;i++){
		flash_write_buffer[i]=i-2;
	}

	flash_SPI_write(0x90,flash_write_buffer,6);

	delay_mili(10);

flash_full_chip_erase();
//	flash_erase(FLASH_BLOCK_ERASE_64KB, flash_write_buffer);
	while(flash_read_status_register(FLASH_READ_STATUS_REGISTER_1)&0x01){
	delay_mili(1000);//wait for erasing flash
	}

	flash_save_data(FLASH_PAGE_PROGRAM,FLASH_READ_BLOCK_3,flash_write_buffer,20);

	delay_mili(10);

	flash_read_data(FLASH_READ_DATA,FLASH_READ_BLOCK_3,flash_write_buffer,20);

	delay_mili(10);

	flash_SPI_write_DMA(FLASH_PAGE_PROGRAM,flash_write_buffer,20);

	delay_mili(10);

	flash_SPI_read(FLASH_READ_DATA,flash_write_buffer,17);

	flash_write_buffer[0]=(FLASH_READ_BLOCK_1>>16)&0xFF;
	flash_write_buffer[1]=(FLASH_READ_BLOCK_1>>8)&0xFF;
	flash_write_buffer[2]=(FLASH_READ_BLOCK_1)&0xFF;

	delay_mili(10);

	flash_SPI_write(FLASH_PAGE_PROGRAM,flash_write_buffer,20);

	delay_mili(10);

	flash_SPI_read(FLASH_READ_DATA,flash_write_buffer,17);


	//end of flash debugging






	while (1) {
		if ((get_Global_Time() - time_flag0_1) >= 30) {
					time_flag0_1 = get_Global_Time();
					pik = 0;
					pek = 0;
				}
				pek++;

				if ((get_Global_Time() - time_flag0_2) >= 1. / FREQUENCY_PID_LOOP) {
					time_flag0_2 = get_Global_Time();
					Ibus_save();
					rewrite_data();


					if (channels[6] < 1400) {
						acro();
						turn_OFF_RED_LED();
						turn_ON_BLUE_LED();
					}


					else if (channels[6] > 1450) {
						stabilize();
						turn_OFF_BLUE_LED();
						turn_ON_RED_LED();
					}

					if (0 != transmitting_is_Done && 0 != New_data_to_send) {
								// Transmit data
								print(table_to_send, ALL_ELEMENTS_TO_SEND);
							}
				}
				else {
					delay_micro(10);

				}

				if ((get_Global_Time() - time_flag0_3) >= 1.f / FREQUENCY_ESC_UPDATE) {
					time_flag0_3 = get_Global_Time();
					update_motors();
					pik++;
				}

				if(USB_detected){
					while(USB_detected){
						turn_OFF_RED_LED();
						turn_ON_BLUE_LED();
						delay_mili(100);
						turn_OFF_BLUE_LED();
						turn_ON_RED_LED();
						delay_mili(100);
					}
				}
	}

}





