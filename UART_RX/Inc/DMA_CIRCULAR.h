#include "stm32f4xx_hal.h"

void USART_IrqHandler (UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

void DMA_IrqHandler (DMA_HandleTypeDef *hdma,UART_HandleTypeDef *huart);


