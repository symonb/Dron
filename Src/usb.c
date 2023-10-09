

#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "flash.h"
#include "usb_device.h"
#include "ff.h"
#include "usb.h" 
#include "scheduler.h"
#include "tasks.h"
#include "sensors/MPU6000.h"

usb_t main_usb = { .class = USB_CLASS_CDC, .status = USB_STATE_NOT_CONNECTED, .connected = false };

void USB_check_connection() {
    // check if usb is connected (check pin value):
    main_usb.connected = (GPIOC->IDR & GPIO_ODR_OD5);
    if (main_usb.connected) {
        if (main_usb.status == USB_STATE_NOT_CONNECTED) {
            main_usb.status = USB_STATE_IDLE;
            add_to_queue(&all_tasks[TASK_USB_HANDLING], &main_scheduler);
        }
    }
    else {
        main_usb.status = USB_STATE_NOT_CONNECTED;
        remove_from_queue(&all_tasks[TASK_USB_HANDLING], &main_scheduler);
    }
};

void usb_communication() {
    switch (main_usb.status)
    {
    case USB_STATE_NOT_CONNECTED:
        break;
    case USB_STATE_IDLE:
        // waiting for some msgs from user
        main_usb.data_to_send_len = 0;
        if (main_usb.data_received_len > 0) {
            main_usb.status = USB_STATE_COMMUNICATION;
        }
        break;
    case USB_STATE_COMMUNICATION:
        //  data are send and reseived via USB
        //  process received data:
        if (strcmp((char*)main_usb.data_received, "help") == 0)
        {
            strcpy((char*)main_usb.data_to_send,
                "\n\r---------HELP---------\n\r"
                "Send commands without newline, sapces, tab.\n\r"
                "For commands with options send:\n\n\r"
                "<cmd><space><option>\n\n\r"
                "list_cmd\t\t list all possible commands\n\r");
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
        }
        else if (strcmp((char*)main_usb.data_received, "list_cmd") == 0)
        {
            strcpy((char*)main_usb.data_to_send,
                "\n\rpossible commands:"
                "\n\rset_mode\t<MODE>\tchange USB mode <MSC, DFU>"
                "\n\rPID_show\t\tshow current PID values"
                "\n\rflash_format\t\terase and format flash memory"
                "\n\racc_level\t\tlevel up drone (set curent attitiude as level)"
                "\n\racc_calibrate\t\tcalibrate accelerometer"
                "\n\rreboot\t\t\treboot drone"
                "\n\rhelp\t\t\thelp info\n\r");
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
        }
        else if (strcmp((char*)main_usb.data_received, "set_mode MSC") == 0) {
            strcpy((char*)main_usb.data_to_send,
                "\n\rsetting MSC mode\n\rconnection will be lost...");
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
            delay_mili(10);
            for (int i = 0; i < TASKS_COUNT;i++) {
                remove_from_queue(&all_tasks[i], &main_scheduler);
            }
            main_usb.status = USB_STATE_NOT_CONNECTED;
            USB_DevDisconnect(USB_OTG_FS);
            USB_ResetPort(USB_OTG_FS);
            MX_USB_DEVICE_Init(USB_CLASS_MSC);
            break;
        }
        else if (strcmp((char*)main_usb.data_received, "set_mode DFU") == 0) {
            strcpy((char*)main_usb.data_to_send,
                "\n\rsetting DFU mode\n\rconnection will be lost...");
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
            delay_mili(10);
            main_usb.status = USB_STATE_NOT_CONNECTED;
            USB_DevDisconnect(USB_OTG_FS);
            USB_ResetPort(USB_OTG_FS);
            MX_USB_DEVICE_Init(USB_CLASS_DFU);
            break;
        }
        else if (strcmp((char*)main_usb.data_received, "PID_show") == 0) {
            snprintf((char*)main_usb.data_to_send, sizeof main_usb.data_to_send, "\n\rcurrent PIDF values:\n\r\tP\tI\tD\tF\n\r"
                "ROLL:\t%.2lf\t%.2lf\t%.2lf\t%.2lf\n\r"
                "PITCH:\t%.2lf\t%.2lf\t%.2lf\t%.2lf\n\r"
                "YAW:\t%.2lf\t%.2lf\t%.2lf\t%.2lf\n\r",
                R_PIDF.P, R_PIDF.I, R_PIDF.D, R_PIDF.F,
                P_PIDF.P, P_PIDF.I, P_PIDF.D, P_PIDF.F,
                Y_PIDF.P, Y_PIDF.I, Y_PIDF.D, Y_PIDF.F);
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));

        }
        else if (strcmp((char*)main_usb.data_received, "reboot") == 0) {
            strcpy((char*)main_usb.data_to_send,
                "\n\rrebooting...\n\r");
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
            delay_mili(10);
            NVIC_SystemReset();
        }
        else if (strcmp((char*)main_usb.data_received, "flash_format") == 0) {
            strcpy((char*)main_usb.data_to_send,
                "\n\rerasing...");

            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
            W25Q128_erase_full_chip();
            while (W25Q128_check_if_busy()) {
                delay_mili(100);
                toggle_BLUE_LED();
                delay_mili(100);
                toggle_RED_LED();
            }
            strcpy((char*)main_usb.data_to_send,
                "\rerasing SUCCEED!\n\r");

            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
            strcpy((char*)main_usb.data_to_send,
                "\n\rformatting...");

            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
            int result = f_format();
            if (result != 0) {
                snprintf((char*)main_usb.data_to_send, sizeof main_usb.data_to_send, "\rformatting ERROR: %d\n\r", result);
            }
            else {
                strcpy((char*)main_usb.data_to_send,
                    "\rformatting SUCCEED!\n\r");
            }
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
        }
        else if (strcmp((char*)main_usb.data_received, "acc_level") == 0) {
            strcpy((char*)main_usb.data_to_send,
                "\n\raccelerometer leveling - don't move a drone\n\r");
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
            delay_mili(10);
            acc_level_calibration(&acc_1);
            snprintf((char*)main_usb.data_to_send, sizeof main_usb.data_to_send,
                "\rleveling SUCCEED!\n\r"
                "new values for q_trans_sensor_to_body_frame:\n\r"
                "w = %lf\n\r"
                "i = %lf\n\r"
                "j = %lf\n\r"
                "k = %lf\n\r",
                q_trans_sensor_to_body_frame.w,
                q_trans_sensor_to_body_frame.x,
                q_trans_sensor_to_body_frame.y,
                q_trans_sensor_to_body_frame.z
            );
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));

        }
        else if (strcmp((char*)main_usb.data_received, "acc_calibrate") == 0) {
            strcpy((char*)main_usb.data_to_send,
                "\n\raccelerometer calibration\n\r");
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
            delay_mili(10);
            char* tab[] = { "set drone NOSE UP, next send \"ok\"",
            "\t- DONE \n\rset drone NOSE DOWN, next send \"ok\"",
            "\t- DONE \n\rset drone SIDE UP, next send \"ok\"",
            "\t- DONE \n\rset drone SIDE DOWN, next send \"ok\"",
            "\t- DONE \n\rset drone BOTTOM UP, next send \"ok\"",
            "\t- DONE \n\rset drone BOTTTOM DOWN, next send \"ok\"",
            "\t- DONE \n\rcollecting data finished - calibration is started\n\r" };
            uint8_t i = 0;
            while (true) {
                strcpy((char*)main_usb.data_to_send, tab[i++]);
                CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));

                while (true) {
                    // wait for user to send "ok"
                    char ok[] = "ok";
                    if (strcmp((char*)main_usb.data_received, (char*)ok) == 0 && main_usb.data_received_len != 0) {
                        main_usb.data_received_len = 0;
                        break;
                    }
                }
                if (acc_calibration(&acc_1) == 1) {
                    strcpy((char*)main_usb.data_to_send, tab[i++]);
                    CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
                    acc_calibration(&acc_1);
                    break;
                }
            }

            snprintf((char*)main_usb.data_to_send, sizeof main_usb.data_to_send,
                "\rcalibration SUCCEED!\n\roffsets:{ %lf, %lf, %lf }\n\r"
                "scales:{ %lf, %lf, %lf }\n\r",
                acc_1.offset[0], acc_1.offset[1], acc_1.offset[2],
                acc_1.scale[0], acc_1.scale[1], acc_1.scale[2]);

            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));

        }
        else {
            strcpy((char*)main_usb.data_to_send, "\n\rreceived:\n\r\"");
            strcat((char*)main_usb.data_to_send, (char*)main_usb.data_received);
            strcat((char*)main_usb.data_to_send, "\"\n\rsend \"help\" for more info\n\r");
            CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));
        }

        //  wait for next data:
        main_usb.data_received_len = 0;
        main_usb.status = USB_STATE_IDLE;
        break;
    default:
        break;
    }

}