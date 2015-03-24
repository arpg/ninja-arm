
#define Encoder1_CHA_GPIO_Pin             GPIO_Pin_1
#define Encoder1_CHB_GPIO_Pin             GPIO_Pin_0
#define Encoder1_GPIO_PORT                GPIOA
#define Encoder1_GPIO_MODE                GPIO_Mode_AF
#define Encoder1_GPIO_PuPd                GPIO_PuPd_NOPULL
#define Encoder1_RCC_AHB1Periph_GPIO      RCC_AHB1Periph_GPIOA
#define Encoder1_TIMPeriod                0xffff
#define Encoder1_TIM_EncoderMode          TIM_EncoderMode_TI12
#define Encoder1_TIM_IC1Polarity          TIM_ICPolarity_Rising
#define Encoder1_TIM_IC2Polarity          TIM_ICPolarity_Rising
#define Encoder1_AutoReloadVal            0xffff
#define Encoder1_SourceA                  GPIO_PinSource1
#define Encoder1_SourceB                  GPIO_PinSource0
#define Encoder1_TIMER                    TIM5
#define Encoder1_GPIO_AF_TIM              GPIO_AF_TIM5
#define Encoder1_Timer_Clock_Enable       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

#define Encoder2_CHA_GPIO_Pin             GPIO_Pin_6
#define Encoder2_CHB_GPIO_Pin             GPIO_Pin_7
#define Encoder2_GPIO_PORT                GPIOC
#define Encoder2_GPIO_MODE                GPIO_Mode_AF
#define Encoder2_GPIO_PuPd                GPIO_PuPd_NOPULL
#define Encoder2_RCC_AHB1Periph_GPIO      RCC_AHB1Periph_GPIOC
#define Encoder2_TIMPeriod                0xffff
#define Encoder2_TIM_EncoderMode          TIM_EncoderMode_TI12
#define Encoder2_TIM_IC1Polarity          TIM_ICPolarity_Rising
#define Encoder2_TIM_IC2Polarity          TIM_ICPolarity_Rising
#define Encoder2_AutoReloadVal            0xffff
#define Encoder2_SourceA                  GPIO_PinSource6
#define Encoder2_SourceB                  GPIO_PinSource7
#define Encoder2_TIMER                    TIM8
#define Encoder2_GPIO_AF_TIM              GPIO_AF_TIM8
#define Encoder2_Timer_Clock_Enable       RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);

#define Encoder3_CHA_GPIO_Pin             GPIO_Pin_6
#define Encoder3_CHB_GPIO_Pin             GPIO_Pin_7
#define Encoder3_GPIO_PORT                GPIOA
#define Encoder3_GPIO_MODE                GPIO_Mode_AF
#define Encoder3_GPIO_PuPd                GPIO_PuPd_NOPULL
#define Encoder3_RCC_AHB1Periph_GPIO      RCC_AHB1Periph_GPIOA
#define Encoder3_TIMPeriod                0xffff
#define Encoder3_TIM_EncoderMode          TIM_EncoderMode_TI12
#define Encoder3_TIM_IC1Polarity          TIM_ICPolarity_Rising
#define Encoder3_TIM_IC2Polarity          TIM_ICPolarity_Rising
#define Encoder3_AutoReloadVal            0xffff
#define Encoder3_SourceA                  GPIO_PinSource6
#define Encoder3_SourceB                  GPIO_PinSource7
#define Encoder3_TIMER                    TIM3
#define Encoder3_GPIO_AF_TIM              GPIO_AF_TIM3
#define Encoder3_Timer_Clock_Enable       RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

#define Encoder4_CHA_GPIO_Pin             GPIO_Pin_9
#define Encoder4_CHB_GPIO_Pin             GPIO_Pin_11
#define Encoder4_GPIO_PORT                GPIOE
#define Encoder4_GPIO_MODE                GPIO_Mode_AF
#define Encoder4_GPIO_PuPd                GPIO_PuPd_NOPULL
#define Encoder4_RCC_AHB1Periph_GPIO      RCC_AHB1Periph_GPIOE
#define Encoder4_TIMPeriod                0xffff
#define Encoder4_TIM_EncoderMode          TIM_EncoderMode_TI12
#define Encoder4_TIM_IC1Polarity          TIM_ICPolarity_Rising
#define Encoder4_TIM_IC2Polarity          TIM_ICPolarity_Rising
#define Encoder4_AutoReloadVal            0xffff
#define Encoder4_SourceA                  GPIO_PinSource11
#define Encoder4_SourceB                  GPIO_PinSource9
#define Encoder4_TIMER                    TIM1
#define Encoder4_GPIO_AF_TIM              GPIO_AF_TIM1
#define Encoder4_Timer_Clock_Enable       RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);


#include "Utilities/InterruptTemplate.h"
#include "Drivers/USART3DMA.h"

struct EncoderPoses
{
    unsigned long int LB;
    unsigned long int LF;
    unsigned long int RB;
    unsigned long int RF;
};

class Encoder : InterruptTemplate
{
public:
    void Config(void);
    void Push2Pack(Transmit_CommandPacket &_data);
    void GetEncoderPoses(EncoderPoses& Position);
private:
    
};