#pragma once
//-----------------------------------------------------------------------------
// InterruptTemplate.h
//
// This file contains the interrupt template class which is used to call
// interrupt ISR functions on derived members
//-----------------------------------------------------------------------------
#include "stm32f2xx_conf.h"
#define enableInterrupts() __enable_irq()
#define disableInterrupts() __disable_irq()

#define SYSTICK_INTERRUPT_HANDLER g_handlersSystick
#define EXTI0_INTERRUPT_HANDLER g_handlersExti0
#define EXTI1_INTERRUPT_HANDLER g_handlersExti1
#define EXTI4_INTERRUPT_HANDLER g_handlersExti4
#define EXTI5_INTERRUPT_HANDLER g_handlersExti5
#define TIM2_INTERRUPT_HANDLER g_handlersTim2
#define I2C1EV_INTERRUPT_HANDLER g_handlersI2c1Ev
#define I2C2EV_INTERRUPT_HANDLER g_handlersI2c2Ev
#define I2C3EV_INTERRUPT_HANDLER g_handlersI2c3Ev
#define USART3_INTERRUPT_HANDLER g_handlersUsart3
#define USART3_INTERRUPT_HANDLER g_handlersSpi1
#define USART3_INTERRUPT_HANDLER g_handlersSpi2

#define DMA1STREAM0_INTERRUPT_HANDLER g_handlersDma1Stream0
#define DMA1STREAM1_INTERRUPT_HANDLER g_handlersDma1Stream1
#define DMA1STREAM2_INTERRUPT_HANDLER g_handlersDma1Stream2
#define DMA1STREAM3_INTERRUPT_HANDLER g_handlersDma1Stream3
#define DMA1STREAM3_INTERRUPT_HANDLER g_handlersDma1Stream4

#define DMA2STREAM0_INTERRUPT_HANDLER g_handlersDma2Stream0
#define DMA2STREAM3_INTERRUPT_HANDLER g_handlersDma2Stream3
#define DMA2STREAM3_INTERRUPT_HANDLER g_handlersDma2Stream4


//-----------------------------------------------------------------------------
// the maximum number of interrupt handlers that can be registered for a single interrupt
//-----------------------------------------------------------------------------
#define MAX_INTERRUPT_HANDLERS 2


class InterruptTemplate
{
public:

    static InterruptTemplate *SYSTICK_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *EXTI0_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *EXTI1_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *EXTI4_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *EXTI5_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *TIM2_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *I2C1EV_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *I2C2EV_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *I2C3EV_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *USART3_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *SPI1_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *SPI2_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];

    static InterruptTemplate *DMA1STREAM0_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *DMA1STREAM1_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *DMA1STREAM2_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *DMA1STREAM3_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *DMA1STREAM4_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];

    static InterruptTemplate *DMA2STREAM0_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *DMA2STREAM3_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
    static InterruptTemplate *DMA2STREAM4_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];


    static void initialiseHandlers();

    //-----------------------------------------------------------------------------
    // This function is used to register to be added to an interrupt template
    //-----------------------------------------------------------------------------
    static bool registerForInterrupt(InterruptTemplate *handlers[MAX_INTERRUPT_HANDLERS],InterruptTemplate *interruptTemplate);


    //-----------------------------------------------------------------------------
    // Functions that are used to automate the interrupt functionality (static)
    //-----------------------------------------------------------------------------
    static void onNotifyInterruptSystick();
    static void onNotifyInterruptExti0();
    static void onNotifyInterruptExti1();
    static void onNotifyInterruptExti4();
    static void onNotifyInterruptExti9_5();
    static void onNotifyInterruptTim2();
    static void onNotifyInterruptI2c1Ev();
    static void onNotifyInterruptI2c2Ev();
    static void onNotifyInterruptI2c3Ev();
    static void onNotifyInterruptUsart3();
    static void onNotifyInterruptSpi1();
    static void onNotifyInterruptSpi2();

    static void onNotifyInterruptDma1Stream0();
    static void onNotifyInterruptDma1Stream1();
    static void onNotifyInterruptDma1Stream2();
    static void onNotifyInterruptDma1Stream3();
    static void onNotifyInterruptDma1Stream4();

    static void onNotifyInterruptDma2Stream0();
    static void onNotifyInterruptDma2Stream3();
    static void onNotifyInterruptDma2Stream4();


    //-----------------------------------------------------------------------------
    // Functions that are to be overridden by derived classes
    //-----------------------------------------------------------------------------
    virtual void onInterruptSystick();
    virtual void onInterruptExti0();
    virtual void onInterruptExti1();
    virtual void onInterruptExti4();
    virtual void onInterruptExti5();
    virtual void onInterruptTim2();
    virtual void onInterruptI2c1Ev();
    virtual void onInterruptI2c2Ev();
    virtual void onInterruptI2c3Ev();
    virtual void onInterruptUsart3();
    virtual void onInterruptSpi1();
    virtual void onInterruptSpi2();

    virtual void onInterruptDma1Stream0();
    virtual void onInterruptDma1Stream1();
    virtual void onInterruptDma1Stream2();
    virtual void onInterruptDma1Stream3();
    virtual void onInterruptDma1Stream4();

    virtual void onInterruptDma2Stream0();
    virtual void onInterruptDma2Stream3();
    virtual void onInterruptDma2Stream4();
    
};

