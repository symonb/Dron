

#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "usbd_cdc_if.h"
#include "usb.h" 

usb_t main_usb = { .class = USB_CLASS_CDC, .status = USB_STATE_NOT_CONNECTED, .connected = false };

void USB_check_connection() {
    // check if usb is connected (check pin value):
    delay_micro(100);
    main_usb.connected = (GPIOA->IDR & GPIO_ODR_OD5);
    if (main_usb.connected) {
        if (main_usb.status == USB_STATE_NOT_CONNECTED) {
            main_usb.status = USB_STATE_IDLE;
        }
    }
    else {
        main_usb.status = USB_STATE_NOT_CONNECTED;
    }
};

void usb_communication(timeUs_t time) {
    switch (main_usb.status)
    {
    case USB_STATE_NOT_CONNECTED:
        break;
    case USB_STATE_IDLE:
        // waiting for some msgs from user
        main_usb.data_to_send_len = 0;
        if (main_usb.data_received_len > 0) {
            main_usb.status = USB_STATE_COMMUNICATION;
            main_usb.data_to_send_len = sprintf((char*)main_usb.data_to_send, "\n\rUSB communication starts...\n\r");
            CDC_Transmit_FS(main_usb.data_to_send, main_usb.data_to_send_len);
        }
        break;
    case USB_STATE_COMMUNICATION:
        //  data are send and reseived via USB
        //  process received data:
        strcpy((char*)main_usb.data_to_send, "received:\n\r\"");
        strcat((char*)main_usb.data_to_send, (char*)main_usb.data_received);
        strcat((char*)main_usb.data_to_send, "\"\n\rdone\n\r");
        CDC_Transmit_FS(main_usb.data_to_send, strlen((char*)main_usb.data_to_send));

        //  wait for next data:
        main_usb.data_received_len = 0;
        main_usb.status = USB_STATE_IDLE;
        break;
    default:
        break;
    }

    CDC_Transmit_FS(main_usb.data_to_send, main_usb.data_to_send_len);
}