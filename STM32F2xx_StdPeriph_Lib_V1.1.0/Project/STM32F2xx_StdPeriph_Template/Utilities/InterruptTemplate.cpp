//-----------------------------------------------------------------------------
// InterruptTemplate.cpp
//
// This file contains the interrupt template class which is used to call
// interrupt ISR functions on derived members
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------
#include "main.h"

#include "interruptTemplate.h"


//-----------------------------------------------------------------------------
// Declaration of static member variables
//-----------------------------------------------------------------------------
InterruptTemplate *	InterruptTemplate::SYSTICK_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::EXTI0_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::EXTI1_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::EXTI4_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::EXTI5_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::TIM2_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::I2C1EV_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::I2C2EV_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::I2C3EV_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::USART3_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::SPI1_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::SPI2_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::DMA1STREAM0_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::DMA1STREAM1_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::DMA1STREAM2_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::DMA1STREAM3_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::DMA1STREAM4_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];

InterruptTemplate *	InterruptTemplate::DMA2STREAM0_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];
InterruptTemplate *	InterruptTemplate::DMA2STREAM3_INTERRUPT_HANDLER[MAX_INTERRUPT_HANDLERS];

//-----------------------------------------------------------------------------
// Initialises all the handlers to zero so that they are not accidentally fired
//-----------------------------------------------------------------------------
void InterruptTemplate::initialiseHandlers()
{
    int i = 0 ;

    for( int j = 0; j < 2 ; j++ )
    {
      int b = 1;
    }

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        SYSTICK_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        EXTI0_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        EXTI1_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        EXTI4_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        EXTI5_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        TIM2_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        I2C1EV_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        I2C2EV_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        I2C3EV_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        USART3_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        SPI1_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        SPI2_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        DMA1STREAM0_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        DMA1STREAM1_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        DMA1STREAM2_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        DMA1STREAM3_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        DMA1STREAM4_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        DMA2STREAM0_INTERRUPT_HANDLER[i] = NULL;

    for (i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
        DMA2STREAM3_INTERRUPT_HANDLER[i] = NULL;
}

//-----------------------------------------------------------------------------
// This function is used to register to be added to an interrupt template
//-----------------------------------------------------------------------------
bool InterruptTemplate::registerForInterrupt(InterruptTemplate *handlers[MAX_INTERRUPT_HANDLERS],InterruptTemplate *interruptTemplate)
{
    for (int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++)
    {
        if( handlers[i] == NULL )
        {
            handlers[i] = interruptTemplate;
            return true;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
// System tick notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptSystick()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( SYSTICK_INTERRUPT_HANDLER[i] != NULL)
            SYSTICK_INTERRUPT_HANDLER[i]->onInterruptSystick();
    }
}


//-----------------------------------------------------------------------------
// EXTI0 notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptExti0()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( EXTI0_INTERRUPT_HANDLER[i] != NULL)
            EXTI0_INTERRUPT_HANDLER[i]->onInterruptExti0();
    }
    EXTI_ClearITPendingBit(EXTI_Line0);
}

//-----------------------------------------------------------------------------
// EXTI1 notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptExti1()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( EXTI1_INTERRUPT_HANDLER[i] != NULL)
            EXTI1_INTERRUPT_HANDLER[i]->onInterruptExti1();
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
}

//-----------------------------------------------------------------------------
// EXTI4 notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptExti4()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( EXTI4_INTERRUPT_HANDLER[i] != NULL)
            EXTI4_INTERRUPT_HANDLER[i]->onInterruptExti4();
    }

    EXTI_ClearITPendingBit(EXTI_Line4);
}

//-----------------------------------------------------------------------------
// EXTI 9 - 5 notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptExti9_5()
{
    //calculate which line it was that was called and call appropriate handles
    if( EXTI_GetITStatus(EXTI_Line5) == SET )
    {
        for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
        {
            if( EXTI5_INTERRUPT_HANDLER[i] != NULL)
                EXTI5_INTERRUPT_HANDLER[i]->onInterruptExti5();
        }

        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}

//-----------------------------------------------------------------------------
// Timer 2 notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptTim2()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( TIM2_INTERRUPT_HANDLER[i] != NULL)
            TIM2_INTERRUPT_HANDLER[i]->onInterruptTim2();
    }
}

//-----------------------------------------------------------------------------
// I2C1 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptI2c1Ev()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( I2C1EV_INTERRUPT_HANDLER[i] != NULL)
            I2C1EV_INTERRUPT_HANDLER[i]->onInterruptI2c1Ev();
    }
}

//-----------------------------------------------------------------------------
// I2C2 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptI2c2Ev()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( I2C2EV_INTERRUPT_HANDLER[i] != NULL)
            I2C2EV_INTERRUPT_HANDLER[i]->onInterruptI2c2Ev();
    }
}

//-----------------------------------------------------------------------------
// I2C3 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptI2c3Ev()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( I2C3EV_INTERRUPT_HANDLER[i] != NULL)
            I2C3EV_INTERRUPT_HANDLER[i]->onInterruptI2c3Ev();
    }
}

//-----------------------------------------------------------------------------
// USART3 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptUsart3()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( USART3_INTERRUPT_HANDLER[i] != NULL)
            USART3_INTERRUPT_HANDLER[i]->onInterruptUsart3();
    }
}

//-----------------------------------------------------------------------------
// SPI1 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptSpi1()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( SPI1_INTERRUPT_HANDLER[i] != NULL)
            SPI1_INTERRUPT_HANDLER[i]->onInterruptSpi1();
    }
}

//-----------------------------------------------------------------------------
// SPI2 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptSpi2()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( SPI1_INTERRUPT_HANDLER[i] != NULL)
            SPI1_INTERRUPT_HANDLER[i]->onInterruptSpi2();
    }
}

//-----------------------------------------------------------------------------
// DMA1 stream 0 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptDma1Stream0()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( DMA1STREAM0_INTERRUPT_HANDLER[i] != NULL)
            DMA1STREAM0_INTERRUPT_HANDLER[i]->onInterruptDma1Stream0();
    }
}

//-----------------------------------------------------------------------------
// DMA1 stream 1 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptDma1Stream1()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( DMA1STREAM1_INTERRUPT_HANDLER[i] != NULL)
            DMA1STREAM1_INTERRUPT_HANDLER[i]->onInterruptDma1Stream1();
    }
}

//-----------------------------------------------------------------------------
// DMA1 stream 2 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptDma1Stream2()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( DMA1STREAM2_INTERRUPT_HANDLER[i] != NULL)
            DMA1STREAM2_INTERRUPT_HANDLER[i]->onInterruptDma1Stream2();
    }
}

//-----------------------------------------------------------------------------
// DMA1 stream 3 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptDma1Stream3()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( DMA1STREAM3_INTERRUPT_HANDLER[i] != NULL)
            DMA1STREAM3_INTERRUPT_HANDLER[i]->onInterruptDma1Stream3();
    }
}

//-----------------------------------------------------------------------------
// DMA1 stream 3 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptDma1Stream4()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( DMA1STREAM4_INTERRUPT_HANDLER[i] != NULL)
            DMA1STREAM4_INTERRUPT_HANDLER[i]->onInterruptDma1Stream4();
    }
}

//-----------------------------------------------------------------------------
// DMA2 stream 0 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptDma2Stream0()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( DMA2STREAM0_INTERRUPT_HANDLER[i] != NULL)
            DMA2STREAM0_INTERRUPT_HANDLER[i]->onInterruptDma2Stream0();
    }
}

//-----------------------------------------------------------------------------
// DMA1 stream 3 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptDma2Stream3()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( DMA2STREAM3_INTERRUPT_HANDLER[i] != NULL)
            DMA2STREAM3_INTERRUPT_HANDLER[i]->onInterruptDma2Stream3();
    }
}

//-----------------------------------------------------------------------------
// DMA1 stream 4 Event notification function. The system calls this function to notify
// the interrupt manager of an interrupt
//-----------------------------------------------------------------------------
void InterruptTemplate::onNotifyInterruptDma2Stream4()
{
    for(int i = 0 ; i < MAX_INTERRUPT_HANDLERS ; i++ )
    {
        if( DMA2STREAM4_INTERRUPT_HANDLER[i] != NULL)
            DMA2STREAM4_INTERRUPT_HANDLER[i]->onInterruptDma2Stream4();
    }
}

//-----------------------------------------------------------------------------
// Empty function definitions
//-----------------------------------------------------------------------------
void InterruptTemplate::onInterruptSystick() {}
void InterruptTemplate::onInterruptExti0() {}
void InterruptTemplate::onInterruptExti1() {}
void InterruptTemplate::onInterruptExti4() {}
void InterruptTemplate::onInterruptExti5() {}
void InterruptTemplate::onInterruptTim2() {}
void InterruptTemplate::onInterruptI2c1Ev() {}
void InterruptTemplate::onInterruptI2c2Ev() {}
void InterruptTemplate::onInterruptI2c3Ev() {}
void InterruptTemplate::onInterruptUsart3() {}
void InterruptTemplate::onInterruptSpi1() {}
void InterruptTemplate::onInterruptSpi2() {}
void InterruptTemplate::onInterruptDma1Stream0() {}
void InterruptTemplate::onInterruptDma1Stream1() {}
void InterruptTemplate::onInterruptDma1Stream2() {}
void InterruptTemplate::onInterruptDma1Stream3() {}
void InterruptTemplate::onInterruptDma1Stream4() {}

void InterruptTemplate::onInterruptDma2Stream0() {}
void InterruptTemplate::onInterruptDma2Stream3() {}
void InterruptTemplate::onInterruptDma2Stream4() {}
