
#include "mine.h"
#include "usart.h"
#include "stm32l4xx_hal.h"


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
