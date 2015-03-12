//#ifndef __USART3DMA_H
//#define __USART3DMA_H
#include "Utilities/InterruptTemplate.h"
class serialdriver:InterruptTemplate{
public:
    void RCC_Configuration(void);
    void GPIO_Configuration(void);
    void USART3_Configuration(void);
    void DMA_Configuration(void);
    //void DMA1_Stream3_IRQHandler(void);
    void DMA1_Stream1_IRQHandler(void);
    void NVIC_Configuration(void);
};
//#endif
