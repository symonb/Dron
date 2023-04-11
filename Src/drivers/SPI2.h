

#ifndef SPI2_H_
#define SPI2_H_

void CS_SPI2_disable();
void CS_SPI2_enable();
void SPI2_disable();
void SPI2_enable();
void SPI2_transmit(const uint8_t* data, uint16_t size);
void SPI2_transmit_one(uint8_t data);
void SPI2_transmit_DMA(uint8_t* data, int size);
void SPI2_receive(uint8_t* data, uint16_t size);
void SPI2_receive_one(uint8_t* data);
bool failsafe_SPI2();

#endif