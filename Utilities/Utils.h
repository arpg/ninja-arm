#ifndef UTILS_H
#define UTILS_H

#include "stm32f2xx.h"
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_gpio.h"

///////////////////////////////////////////////////////////////////////////////
struct GPIO_Pin
{
    GPIO_TypeDef* m_pGpio;
    uint32_t m_nGpioClk;
    uint32_t m_nPin;
    uint32_t m_nPinSource;
    uint32_t m_nPinAF;

    GPIO_Pin() :
        m_pGpio(0),
        m_nGpioClk(0),
        m_nPin(0),
        m_nPinSource(0),
        m_nPinAF(0)
    {}

    GPIO_Pin(GPIO_TypeDef* gpio, const uint32_t gpioClk, const uint32_t pin) :
        m_pGpio(gpio),
        m_nGpioClk(gpioClk),
        m_nPin(pin),
        m_nPinSource(0),
        m_nPinAF(0)
    {}

    GPIO_Pin(GPIO_TypeDef* gpio, const uint32_t gpioClk, const uint32_t pin, const uint32_t pinSource, const uint32_t pinAF ) :
        m_pGpio(gpio),
        m_nGpioClk(gpioClk),
        m_nPin(pin),
        m_nPinSource(pinSource),
        m_nPinAF(pinAF)
    {}
};


///////////////////////////////////////////////////////////////////////////////
int RCCSetClock(const void* pPeriph, const uint32_t nClockId, const FunctionalState NewState);

#endif // UTILSX_H
