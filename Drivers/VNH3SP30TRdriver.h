#ifndef VNH3SP30TRDRIVER_H
#define VNH3SP30TRDRIVER_H

#include "Utilities/Utils.h"

class VNH3SP30TRDriver
{
public:
    enum Mode{
        eModeForward,
        eModeReverse,
        eModeBrakeGND,
        eModeBrakeVCC
    };

    VNH3SP30TRDriver();
    void Initialize(TIM_TypeDef* pPwmTimer,
                    const uint32_t pwmTimerClk,
                    TIM_TypeDef *pPwmTimer2,
                    const uint32_t pwmTimerClk2,
                    TIM_TypeDef *pPwmTimer3,
                    const uint32_t pwmTimerClk3,
                    const GPIO_Pin inAPin,
                    const GPIO_Pin inBPin,
                    const GPIO_Pin pwmPin,
                    const GPIO_Pin inAPin2,
                    const GPIO_Pin inBPin2,
                    const GPIO_Pin pwmPin2,
                    const GPIO_Pin inAPin3,
                    const GPIO_Pin inBPin3,
                    const GPIO_Pin pwmPi3);

    void SetMode(const Mode eMode);
    void SetSpeed(const float nSpeed);
    static uint16_t m_nPwmPeriod;

private:
    GPIO_Pin m_InAPin;
    GPIO_Pin m_InBPin;
    GPIO_Pin m_PwmPin;
    TIM_TypeDef* m_nTimer;

    GPIO_Pin m_InAPin2;
    GPIO_Pin m_InBPin2;
    GPIO_Pin m_PwmPin2;
    TIM_TypeDef* m_nTimer2;

    GPIO_Pin m_InAPin3;
    GPIO_Pin m_InBPin3;
    GPIO_Pin m_PwmPin3;
    TIM_TypeDef* m_nTimer3;

};

#endif // VNH3SP30TRDRIVER_H
