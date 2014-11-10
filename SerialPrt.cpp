#include "SerialPrt.h"

void SerialPrtDriver::configureUart()
{
//UART1 Configuration
//1. System Clock confirm
//sample code is base on 72MHz to setting
/* #define SYSCLK_FREQ_HSE    HSE_VALUE */
#define SYSCLK_FREQ_72MHz  72000000
uint32_t SystemCoreClock         = SYSCLK_FREQ_72MHz;
}

void Initialize(GPIO_Pin txPin, GPIO_Pin rxPin, GPIO_Pin ctsPin, GPIO_Pin rtsPin);
{
  //3. GPIO Setting for GPIOA Pin 9, 10
  /* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = rxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART1 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = txPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

4. Setting GPIO Clock
  /* Enable USART1, GPIOA, GPIOD and AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 |      RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB );
}


void SerialPrtDriver::NVIC_Conf()
{
  //2. Setting NVIC Configuration
  /* Set the Vector Table base location at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);  
/*Enable USART1 interrupt*/
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}




5. UART Configuration
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; /*send a idle frame,cause TXE=1 interupt*/
  USART_InitStructure.USART_Clock = USART_Clock_Disable;
  USART_InitStructure.USART_CPOL = USART_CPOL_Low;
  USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
  USART_InitStructure.USART_LastBit = USART_LastBit_Disable;
 
  /* Configure USART1 */
  USART_Init(USART1, &USART_InitStructure);




6. Enable UART 1 Interrupt
  /* Enable the USART Transmoit interrupt: this interrupt is generated when the
   USART1 transmit data register is empty */ 
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);   

/* Enable the USART Receive interrupt: this interrupt is generated when the
   USART1 receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);





7. Enable UART1
  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);

8. Creater USART1_IRQHandler() function in stm32f10x_it.c
void USART1_IRQHandler(void)
{
   if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    unsigned char recdata;
    unsigned char temphead;
    /* Clear the USART1 Receive interrupt */
     USART_ClearITPendingBit(USART1, USART_IT_RXNE);
     recdata=USART_ReceiveData(USART1)&0xFF;
     temphead=(UART1_RxHead+1)&0xFF;
     UART1_RxHead=temphead;
     UART1_RX_BUFFER[temphead]=recdata;
  }
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {  
    unsigned char temptail;
    USART_ClearITPendingBit(USART1, USART_IT_TXE);
    if(UART1_TxHead==UART1_TxTail)
    {
        Flag_UART1_SendOK=0;
    }
    else     
   {   
        temptail= UART1_TxTail+1;
        UART1_TxTail=temptail;         */
        USART_SendData(USART1, UART1_TX_BUFFER[temptail]);         
    }
    if(TxCounter == NbrOfDataToTransfer)
    {
   } 
 }

9. Printf function
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

add function
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(EVAL_COM1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
  {}
  return ch;
}