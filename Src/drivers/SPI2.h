

#ifndef SPI2_H_
#define SPI2_H_
void SPI2_enable();
void SPI2_disable();
void SPI2_CS_enable();
void SPI2_CS_disable();
void SPI2_transmit(const uint8_t* data, uint16_t size);
void SPI2_transmit_DMA(uint8_t* data, uint16_t size);
void SPI2_transmit_one(uint8_t data);
void SPI2_receive(uint8_t* data, uint16_t size);
void SPI2_receive_one(uint8_t* data);
bool SPI2_failsafe();
#endif