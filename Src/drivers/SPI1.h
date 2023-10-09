
#ifndef SPI1_H_
#define SPI1_H_
void SPI1_enable();
void SPI1_disable();
void SPI1_CS_enable();
void SPI1_CS_disable();
void SPI1_transmit(const uint8_t* data, int size);
void SPI1_receive(uint8_t* data, int size);
void SPI1_receive_DMA(uint8_t* data, uint16_t size);
bool SPI1_failsafe();
#endif
