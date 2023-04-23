
#ifndef USB_H_
#define USB_H_

#include "global_variables.h"

void USB_check_connection();
void usb_communication(timeUs_t time);

typedef enum
{
    USB_STATE_NOT_CONNECTED,
    USB_STATE_STARTUP,
    USB_STATE_COMMUNICATION,
    USB_STATE_IDLE,
    USB_COUNT
} usb_state_e;

typedef struct
{
    bool connected;
    usb_state_e status;
    uint8_t data_to_send[50];
    uint8_t data_to_send_len;
    uint8_t data_received[50];
    uint8_t data_received_len;
}usb_t;

extern usb_t main_usb;


#endif /* USB_H_ */

