#ifndef __MINE_H__
#define __MINE_H__

#include "main.h"

ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint32_t USART_IT);
void USART_ITConfig(USART_TypeDef* USARTx, uint32_t USART_IT, FunctionalState NewState);

int get_data(uint8_t* data);

#endif
