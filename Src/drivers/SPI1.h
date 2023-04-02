
#ifndef SPI1_H_
#define SPI1_H_
void SPI1_enable();
void SPI1_disable();
void CS_SPI1_enable();
void CS_SPI1_disable();
void SPI1_transmit(uint8_t* data, int size);
void SPI1_receive(uint8_t* data, int size);
void SPI1_receive_DMA(uint8_t* data, uint16_t size);
bool failsafe_SPI1();
#endif
