#ifndef SPI3_H_
#define SPI3_H_

void CS_SPI3_disable();
void CS_SPI3_enable();
void SPI3_enable();
void SPI3_disable();
void SPI3_transmit(const uint8_t* data, uint8_t size);
void SPI3_transmit_DMA(uint8_t* data, int size);
void SPI3_receive(uint8_t* data, int size);
void SPI3_receive_DMA(uint8_t* data, int size);
bool failsafe_SPI3();

#endif