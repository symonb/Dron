#ifndef SPI3_H_
#define SPI3_H_

void SPI3_CS_disable();
void SPI3_CS_enable();
void SPI3_enable();
void SPI3_disable();
void SPI3_transmit(const uint8_t* data, int size);
void SPI3_transmit_DMA(uint8_t* data, int size);
void SPI3_receive(uint8_t* data, int size);
void SPI3_receive_DMA(uint8_t* data, int size);
bool SPI3_failsafe();

#endif