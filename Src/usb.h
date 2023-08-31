
#ifndef USB_H_
#define USB_H_

#include "global_variables.h"

void USB_check_connection();
void usb_communication(timeUs_t time);

typedef enum {
    USB_CLASS_CDC,
    USB_CLASS_DFU,
    USB_CLASS_MSC
}usb_class_e;

typedef enum
{
    USB_STATE_NOT_CONNECTED,
    USB_STATE_COMMUNICATION,
    USB_STATE_IDLE,
    USB_COUNT
} usb_state_e;

typedef struct
{
    usb_class_e class;
    bool connected;
    usb_state_e status;
    uint8_t data_to_send[250];
    uint8_t data_to_send_len;
    uint8_t data_received[150];
    uint8_t data_received_len;
}usb_t;

extern usb_t main_usb;

#endif /* USB_H_ */







