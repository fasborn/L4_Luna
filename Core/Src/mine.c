
#include "mine.h"
#include "usart.h"
#include "stm32l4xx_hal.h"

/*!< USART Interrupts mask */
#define IT_MASK                   ((uint32_t)0x000000FF)

/**
  * @brief  Enables or disables the specified USART interrupts.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt sources to be enabled or disabled.
  *         This parameter can be one of the following values:
  *         @arg USART_IT_WU:  Wake up interrupt.
  *         @arg USART_IT_CM:  Character match interrupt.
  *         @arg USART_IT_EOB:  End of block interrupt.
  *         @arg USART_IT_RTO:  Receive time out interrupt.
  *         @arg USART_IT_CTS:  CTS change interrupt.
  *         @arg USART_IT_LBD:  LIN Break detection interrupt.
  *         @arg USART_IT_TXE:  Tansmit Data Register empty interrupt.
  *         @arg USART_IT_TC:  Transmission complete interrupt.
  *         @arg USART_IT_RXNE:  Receive Data register not empty interrupt.
  *         @arg USART_IT_IDLE:  Idle line detection interrupt.
  *         @arg USART_IT_PE:  Parity Error interrupt.
  *         @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @param  NewState: new state of the specified USARTx interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_ITConfig(USART_TypeDef* USARTx, uint32_t USART_IT, FunctionalState NewState)
{
  uint32_t usartreg = 0, itpos = 0, itmask = 0;
  uint32_t usartxbase = 0;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CONFIG_IT(USART_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  usartxbase = (uint32_t)USARTx;

  /* Get the USART register index */
  usartreg = (((uint16_t)USART_IT) >> 0x08);

  /* Get the interrupt position */
  itpos = USART_IT & IT_MASK;
  itmask = (((uint32_t)0x01) << itpos);

  if (usartreg == 0x02) /* The IT is in CR2 register */
  {
    usartxbase += 0x04;
  }
  else if (usartreg == 0x03) /* The IT is in CR3 register */
  {
    usartxbase += 0x08;
  }
  else /* The IT is in CR1 register */
  {
  }
  if (NewState != DISABLE)
  {
    *(__IO uint32_t*)usartxbase  |= itmask;
  }
  else
  {
    *(__IO uint32_t*)usartxbase &= ~itmask;
  }
}




/**
  * @brief  Checks whether the specified USART interrupt has occurred or not.
  * @param  USARTx: Select the USART peripheral. This parameter can be one of the 
  *         following values: USART1 or USART2 or USART3 or UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt source to check.
  *         This parameter can be one of the following values:
  *         @arg USART_IT_WU:  Wake up interrupt.
  *         @arg USART_IT_CM:  Character match interrupt.
  *         @arg USART_IT_EOB:  End of block interrupt.
  *         @arg USART_IT_RTO:  Receive time out interrupt.
  *         @arg USART_IT_CTS:  CTS change interrupt.
  *         @arg USART_IT_LBD:  LIN Break detection interrupt.
  *         @arg USART_IT_TXE:  Tansmit Data Register empty interrupt.
  *         @arg USART_IT_TC:  Transmission complete interrupt.
  *         @arg USART_IT_RXNE:  Receive Data register not empty interrupt.
  *         @arg USART_IT_IDLE:  Idle line detection interrupt.
  *         @arg USART_IT_ORE:  OverRun Error interrupt.
  *         @arg USART_IT_NE:  Noise Error interrupt.
  *         @arg USART_IT_FE:  Framing Error interrupt.
  *         @arg USART_IT_PE:  Parity Error interrupt.
  * @retval The new state of USART_IT (SET or RESET).
  */
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint32_t USART_IT)
{
  uint32_t bitpos = 0, itmask = 0, usartreg = 0;
  ITStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_GET_IT(USART_IT)); 
  
  /* Get the USART register index */
  usartreg = (((uint16_t)USART_IT) >> 0x08);
  /* Get the interrupt position */
  itmask = USART_IT & IT_MASK;
  itmask = (uint32_t)0x01 << itmask;
  
  if (usartreg == 0x01) /* The IT  is in CR1 register */
  {
    itmask &= USARTx->CR1;
  }
  else if (usartreg == 0x02) /* The IT  is in CR2 register */
  {
    itmask &= USARTx->CR2;
  }
  else /* The IT  is in CR3 register */
  {
    itmask &= USARTx->CR3;
  }
  
  bitpos = USART_IT >> 0x10;
  bitpos = (uint32_t)0x01 << bitpos;
  bitpos &= USARTx->ISR;
  if ((itmask != (uint16_t)RESET)&&(bitpos != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  
  return bitstatus;  
}

int HEADER=0x59; //frame header of data package
int uart[9]; //save data measured by LiDAR
int check = 0; //save check value
int dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature;
uint8_t k = 0;

int get_data(uint8_t* data){
	if(data[0] == HEADER) { //assess data package frame header 0x59
		if (data[1] == HEADER){ //assess data package frame header 0x59
			
			check = data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7];
			k = check&0xff;			
			if (data[8] == k){ //verify the received data as per protocol
				dist = data[2] + data[3] * 256; //calculate distance value
				strength = data[4] + data[5] * 256; //calculate signal strength value
				temprature = data[6] + data[7] *256;//calculate chip temprature
				temprature = temprature/8 - 256;
				
				return dist;
			}
		}
	}
	return 0;
}

uint8_t temp;

//int get_data_test(UART_HandleTypeDef *huart, uint8_t* data){
//   if(HAL_UART_Receive_IT(huart, &temp, 1) == HEADER) { //assess data package frame header 0x59
//     uart[0]=HEADER;
//     if (HAL_UART_Receive_IT(huart, &temp, 1) == HEADER) { //assess data package frame header 0x59
//       uart[1] = HEADER;
//       for (int i = 2; i < 9; i++) { //save data in array
//				uart[i] = HAL_UART_Receive_IT(huart, &temp, 1);
//       }
//       check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
//       if (uart[8] == (check & 0xff)){ //verify the received data as per protocol
//         dist = uart[2] + uart[3] * 256; //calculate distance value
//         strength = uart[4] + uart[5] * 256; //calculate signal strength value
//         temprature = uart[6] + uart[7] *256;//calculate chip temprature
//         temprature = temprature/8 - 256;
//         Serial.print("dist = ");
//         Serial.print(dist); //output measure distance value of LiDAR
//         Serial.print('\t');Serial.print("strength = ");
//         Serial.print(strength); //output signal strength value
//         Serial.print("\t Chip Temprature = ");
//         Serial.print(temprature);
//         Serial.println(" celcius degree"); //output chip temperature of Lidar
//       }
//     }
//   }
//}

uint8_t t_data[9];

HAL_StatusTypeDef newos(void){
  while(1){
    HAL_UART_Receive(&huart2, t_data, 2, 1000);
		while(huart2.RxXferCount>0){};
    if((t_data[0] == HEADER) && (t_data[1] == HEADER)){
      return HAL_UART_Receive(&huart2, t_data, 7, 1000);
    }
  }
}
