

#ifndef IBUS_H_
#define IBUS_H_

void USART1_IRQHandler();
void DMA2_Stream5_IRQHandler();
bool Ibus_save(rx_t* receiver);

#endif /* IBUS_H_ */
