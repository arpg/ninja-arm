#include "USART3DMA.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_usart.h"
#include "stm32f2xx_dma.h"
/**************************************************************************************/
 
void serialdriver::RCC_Configuration(void)
{
  /* --------------------------- System Clocks Configuration -----------------*/
  /* USART3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
 
  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
 
  /* DMA1 clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
}
 
/**************************************************************************************/
 
void serialdriver::GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
 
  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PD.8 USART3_TX, PD.9 USART3_RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  /* Connect USART pins to AF */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
}
 
/**************************************************************************************/
 
void serialdriver::USART3_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;
 
  /* USARTx configuration ------------------------------------------------------*/
  /* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
 
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  USART_Init(USART3, &USART_InitStructure);
 
  USART_Cmd(USART3, ENABLE);
}
 
/**************************************************************************************/
 
// Output as a loop, received data overwrites, and subsequently outputs during next cycle
 
 
void serialdriver::DMA_Configuration( char *_tx_data, char *_rx_data, int tx_size, int rx_size)
{
  DMA_InitTypeDef  DMA_InitStructure;
 
  // USART3_TX DMA Channel 4, DMA1, Stream3
 
  DMA_DeInit(DMA1_Stream3);
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // Transmit
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_tx_data;
  DMA_InitStructure.DMA_BufferSize = tx_size - 1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
 
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);
 
  /* Enable the USART Tx DMA request */
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
 
  /* Enable DMA Stream Transfer Complete interrupt */
  //DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
 
  /* Enable the DMA TX Stream */
  DMA_Cmd(DMA1_Stream3, ENABLE);
 
  // USART3_RX DMA Channel 4, DMA1, Stream1
 
  DMA_DeInit(DMA1_Stream1);
 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Receive
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_rx_data;
  DMA_InitStructure.DMA_BufferSize = rx_size - 1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
 
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
 
  /* Enable the USART Rx DMA request */
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
 
  /* Enable DMA Stream Transfer Complete interrupt */
  //DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
 
  /* Enable the DMA RX Stream */
  DMA_Cmd(DMA1_Stream1, ENABLE);

}
 
/**************************************************************************************/

void serialdriver::DMA1_Stream3_IRQHandler(void) // USART3_TX
{
  // Test on DMA Stream Transfer Complete interrupt 
  if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
  {
    // Clear DMA Stream Transfer Complete interrupt pending bit 
    DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
  }
}

/**************************************************************************************/

void serialdriver::DMA1_Stream1_IRQHandler(void) // USART3_RX
{
  // Test on DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
  {
    // Clear DMA Stream Transfer Complete interrupt pending bit
    DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
  }
}

/**************************************************************************************/
 
void serialdriver::NVIC_Configuration(void)
{
  //and now register this class to receive the RX and TX interrupts
  InterruptTemplate::initialiseHandlers();
  InterruptTemplate::registerForInterrupt(DMA1STREAM1_INTERRUPT_HANDLER,this);
  InterruptTemplate::registerForInterrupt(DMA1STREAM3_INTERRUPT_HANDLER,this);

  NVIC_InitTypeDef NVIC_InitStructure; 
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
   //Enable the USART3 TX DMA Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  /* Enable the USART3 RX DMA Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
 
/**************************************************************************************/
