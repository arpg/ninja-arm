#include "VNH3SP30TRdriver.h"
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_gpio.h"
//#include <algorithm.h>

uint16_t VNH3SP30TRDriver::m_nPwmPeriod = 500;

///////////////////////////////////////////////////////////////////////////////
VNH3SP30TRDriver::VNH3SP30TRDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
void VNH3SP30TRDriver::Initialize(TIM_TypeDef* pPwmTimer,
                                  const uint32_t pwmTimerClk,
                                  const GPIO_Pin inAPin,
                                  const GPIO_Pin inBPin,
                                  const GPIO_Pin inAPin2,
                                  const GPIO_Pin inBPin2,
                                  const GPIO_Pin inAPin3,
                                  const GPIO_Pin inBPin3,
                                  const GPIO_Pin pwmPin
                                  )

{
    m_InAPin = inAPin;
    m_InBPin = inBPin;
    m_PwmPin = pwmPin;
    m_nTimer = pPwmTimer;

    m_InAPin2 = inAPin2;
    m_InBPin2 = inBPin2;

    m_InAPin3 = inAPin3;
    m_InBPin3 = inBPin3;

    //initialize the control GPIOs
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    //enable the GPIO clocks
    RCCSetClock(inAPin.m_pGpio,inAPin.m_nGpioClk, ENABLE);
    RCCSetClock(inBPin.m_pGpio,inBPin.m_nGpioClk, ENABLE);
    
    RCCSetClock(inAPin2.m_pGpio,inAPin2.m_nGpioClk, ENABLE);
    RCCSetClock(inBPin2.m_pGpio,inBPin2.m_nGpioClk, ENABLE);
    
    RCCSetClock(inAPin3.m_pGpio,inAPin3.m_nGpioClk, ENABLE);
    RCCSetClock(inBPin3.m_pGpio,inBPin3.m_nGpioClk, ENABLE);
    
    //configure the GPIO pins    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;

    GPIO_InitStructure.GPIO_Pin = inAPin.m_nPin;
    GPIO_Init(inAPin.m_pGpio, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = inBPin.m_nPin;
    GPIO_Init(inBPin.m_pGpio, &GPIO_InitStructure);    
    
    GPIO_InitStructure.GPIO_Pin = inAPin2.m_nPin;
    GPIO_Init(inAPin2.m_pGpio, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = inBPin2.m_nPin;
    GPIO_Init(inBPin2.m_pGpio, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = inAPin3.m_nPin;
    GPIO_Init(inAPin3.m_pGpio, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = inBPin3.m_nPin;
    GPIO_Init(inBPin3.m_pGpio, &GPIO_InitStructure);

    //configure the PWM pin
    //enable the GPIO clocks
    RCCSetClock(pwmPin.m_pGpio,pwmPin.m_nGpioClk, ENABLE);

    //configure the GPIO pins   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = pwmPin.m_nPin;
    GPIO_Init(pwmPin.m_pGpio, &GPIO_InitStructure);    
    
    RCC_APB2PeriphClockCmd(pwmTimerClk,ENABLE);
    //configure the timer
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    // Compute the prescaler value
    uint16_t prescalerValue = (uint16_t)((RCC_Clocks.PCLK2_Frequency) / 5000000);
    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = m_nPwmPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(pPwmTimer, &TIM_TimeBaseStructure);
    TIM_Cmd(pPwmTimer, ENABLE);

    //Initialize pwm channels
    TIM_OCInitTypeDef OCInitStructure;
    TIM_OCStructInit(&OCInitStructure);
    // PWM1 Mode configuration: Channel1
    OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    OCInitStructure.TIM_Pulse = 250;
    OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(pPwmTimer, &OCInitStructure);
    TIM_OC1PreloadConfig(pPwmTimer, TIM_OCPreload_Enable);
    //configure pin AF
    GPIO_PinAFConfig(pwmPin.m_pGpio, pwmPin.m_nPinSource, pwmPin.m_nPinAF);

}

///////////////////////////////////////////////////////////////////////////////
void VNH3SP30TRDriver::SetMode(const VNH3SP30TRDriver::Mode eMode)
{
    switch(eMode){
        case eModeForward:
            GPIO_ResetBits(m_InBPin.m_pGpio,m_InBPin.m_nPin);
            GPIO_ResetBits(m_InBPin2.m_pGpio,m_InBPin2.m_nPin);
            GPIO_ResetBits(m_InBPin3.m_pGpio,m_InBPin3.m_nPin);
            GPIO_SetBits(m_InAPin2.m_pGpio,m_InAPin2.m_nPin);
            GPIO_SetBits(m_InAPin.m_pGpio,m_InAPin.m_nPin);
            GPIO_SetBits(m_InAPin3.m_pGpio,m_InAPin3.m_nPin);
            break;

        case eModeReverse:
            
            GPIO_ResetBits(m_InAPin.m_pGpio,m_InAPin.m_nPin);
            GPIO_ResetBits(m_InAPin2.m_pGpio,m_InAPin2.m_nPin);
            GPIO_ResetBits(m_InAPin3.m_pGpio,m_InAPin3.m_nPin);
            GPIO_SetBits(m_InBPin.m_pGpio,m_InBPin.m_nPin);
            GPIO_SetBits(m_InBPin2.m_pGpio,m_InBPin2.m_nPin);
            GPIO_SetBits(m_InBPin3.m_pGpio,m_InBPin3.m_nPin);
            break;

        case eModeBrakeGND:
            GPIO_ResetBits(m_InAPin.m_pGpio,m_InAPin.m_nPin);
            GPIO_ResetBits(m_InBPin.m_pGpio,m_InBPin.m_nPin);

            GPIO_ResetBits(m_InAPin2.m_pGpio,m_InAPin2.m_nPin);
            GPIO_ResetBits(m_InBPin2.m_pGpio,m_InBPin2.m_nPin);

            GPIO_ResetBits(m_InAPin3.m_pGpio,m_InAPin3.m_nPin);
            GPIO_ResetBits(m_InBPin3.m_pGpio,m_InBPin3.m_nPin);
            break;

        case eModeBrakeVCC:
            GPIO_SetBits(m_InAPin.m_pGpio,m_InAPin.m_nPin);
            GPIO_SetBits(m_InBPin.m_pGpio,m_InBPin.m_nPin);

            GPIO_SetBits(m_InAPin2.m_pGpio,m_InAPin2.m_nPin);
            GPIO_SetBits(m_InBPin2.m_pGpio,m_InBPin2.m_nPin);

            GPIO_SetBits(m_InAPin3.m_pGpio,m_InAPin3.m_nPin);
            GPIO_SetBits(m_InBPin3.m_pGpio,m_InBPin3.m_nPin);
        break;
    }
}

///////////////////////////////////////////////////////////////////////////////
void VNH3SP30TRDriver::SetSpeed(const float nSpeed)
{

    //int nClampedSpeed = std::max(0,std::min((int)m_nPwmPeriod,(int)(nSpeed*(float)m_nPwmPeriod)));
    int nClampedSpeed = (int)(nSpeed*(float)m_nPwmPeriod);
    float dClampedPos;
    if((int)m_nPwmPeriod>(int)(nSpeed*(float)m_nPwmPeriod))
      nClampedSpeed = (int)(nSpeed*(float)m_nPwmPeriod);
    else
      nClampedSpeed = (int)m_nPwmPeriod;

    if(nClampedSpeed<0)
      nClampedSpeed = 0;

    
    TIM_SetCompare1(m_nTimer, nClampedSpeed);

    //TIM_SetCompare1(m_nTimer2, nClampedSpeed);

    //TIM_SetCompare1(m_nTimer3, nClampedSpeed);
}
