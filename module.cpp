//-----------------------------------------------------------------------------
// module.cpp
//
// This class contains the majority of the functionality driving the IMU module.
// It will automate all functionality of the module, initialize the peripherals
// and sensors and relay information back to the GS.
//
// Author: Nima Keivan
// Revision: Sina Aghli
//-----------------------------------------------------------------------------
#include "main.h"
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_gpio.h"
#include "misc.h"
//#include "Utils.h"

#include <string.h>
//#include "imuKalman.h"
#include "interruptTemplate.h"
#include "UartPacketDriver.h"

//#include "LSM303Driver.h"
//#include "IMU3000Driver.h"
//#include "VenusGpsDriver.h"
#include "stm32f2xx_adc.h"
#include "module.h"
//#include <algorithm>

uint32_t Module::m_nPwmFreq = 200000;
uint16_t Module::m_nPwmPeriod = 0.006 / (1.0/(double)Module::m_nPwmFreq);//0.02 / (1.0/(double)Module::m_nPwmFreq);
uint16_t Module::m_nPwmMin = 0.00029 / (1.0/(double)Module::m_nPwmFreq);
uint16_t Module::m_nPwmMax = 0.00062 / (1.0/(double)Module::m_nPwmFreq);
uint16_t Module::m_nPwmMid = 0.001 / (1.0/(double)Module::m_nPwmFreq);

__IO uint16_t ADC1ConvertedValue[6] = {0,0,0,0,0,0};
__IO uint32_t ADC1ConvertedVoltage = 0;

// ADC1_DR_ADDRESS register address
// refer to http://www.st.com/web/en/resource/technical/document/reference_manual/CD00225773.pdf
// memory map section for more information
#define ADC1_DR_ADDRESS    ((uint32_t)0x4001204C)       

//-----------------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------------
Module::Module()
{
}

//-----------------------------------------------------------------------------
// The main function to contain the infinite loop. This should be called externally
// to prepare system functions.
//-----------------------------------------------------------------------------
void Module::Run()
{
    float val = 0.0;
    int LoopCounter = 1;
    ADC_SoftwareStartConv(ADC1);
    
    while(1)
    {
        unsigned char pPacketData[256];
        memset(pPacketData, 0, 256);
        short nLengthOut=5;
        
////////////////////////////////////////////////////////////////
//  Transmit New Package
////////////////////////////////////////////////////////////////
        Transmit_CommandPacket TCmdPack;
        m_MPU9150Driver.Push2Pack(TCmdPack);
        MyEncoders.Push2Pack(TCmdPack);
        ADC_Push2Pack(TCmdPack);
        m_ComsDriver.AddChecksum(TCmdPack);
        m_ComsDriver.beginWritePacket(&TCmdPack.m_cDelimiter1,TCmdPack.m_cSize);
          
////////////////////////////////////////////////////////////////
//  Receive New Package and apply commands
////////////////////////////////////////////////////////////////
        if( m_ComsDriver.ReadPacket(pPacketData,nLengthOut) == true) {
            CommandPacket* pPacket = (CommandPacket*)pPacketData;
            if(m_ComsDriver.ChkChksum(pPacket))
            {
                SetServoPos(0,(double)pPacket->m_nSteering/500.0);

                if(pPacket->m_nSpeed > 250.0){
                    //SetRgbLed(0,0,true);
                    m_MainMotorDriver.SetMode(VNH3SP30TRDriver::eModeForward);
                    m_MainMotorDriver.SetSpeed(((double)pPacket->m_nSpeed-250)/250.0);
                }else{
                    //SetRgbLed(true,0,0);
                    m_MainMotorDriver.SetMode(VNH3SP30TRDriver::eModeReverse);
                    m_MainMotorDriver.SetSpeed((250.0 -(double)pPacket->m_nSpeed)/250.0);
                }
            }
        }

        if(LoopCounter > 100)
        {
          SetRgbLed(true,false,false);
            LoopCounter = -1;
        }else if(LoopCounter < -10)
        {
          SetRgbLed(false,false,false);
          LoopCounter = 1;
        }else if(LoopCounter<0)
          LoopCounter--;
        else if(LoopCounter>0)
          LoopCounter++;

////////////////////////////////////////////////////////////////
//  Test Functions for Debug
////////////////////////////////////////////////////////////////
/*
        for(int ii=0;ii<360;ii++)
        {
          SetServoPos(0,(sin(val*0.0174)));
//        m_MainMotorDriver.SetSpeed((sin(val)+1.0)/4.0);
          val += 1;
          Delay_ms(5);
        }
*/
        ///////////////////////////////// USART Transmit package

/*
        CommandPacket CMDPACK;
        CMDPACK.m_nSteering = 65532;
        CMDPACK.m_nSpeed = -12345;
        m_ComsDriver.beginWritePacket(&CMDPACK.m_cDelimiter1,(short int)CMDPACK.m_cSize);
*/
        ///////////////////////////////// ADC Test
        //int Pot1Value = (int)ADC1->DR;//ADC1ConvertedValue;
        //int Pot2Value = ADC1ConvertedValue[0];
        
        //////////////////////////////// PWM Test
        /*for(int j=0; j<10000; j++)
        {
          SetServoPos(0,((double)j/10000));
          for(int ii = 0; ii < 1000; ii++){}
        }*/
        
        /////////////////////////////// Test Motor
        /*
        m_MainMotorDriver.SetMode(VNH3SP30TRDriver::eModeForward);
        for(int j=10000; j>5; j--)
        {
          m_MainMotorDriver.SetSpeed(((float)j/10000));
          for(int ii = 0; ii < 10000; ii++){}
        }
        //m_MainMotorDriver.SetMode(VNH3SP30TRDriver::eModeReverse);
        for(int j=5; j<10000; j++)
        {
          m_MainMotorDriver.SetSpeed(((float)j/10000));
          for(int ii = 0; ii < 10000; ii++){}
        }
        //m_MainMotorDriver.SetSpeed(0.2f);
        */
        ///////////////////////////////// Test MPU9150
         // char TempData[21];
          //Enable DataReady Interrupt of sensor
          //m_MPU9150Driver.getRegister(0x38,TempData);
          
         // m_MPU9150Driver.getRegister(0x38,TempData);

          
          //m_MPU9150Driver.setRegister(MPU9150_Ext_IRQen,1);
          //MPU9150_Data_Structure IMU;
          //m_MPU9150Driver.getData(IMU);
          //m_MPU9150Driver.getRegister(MPU9150_ACCEL_XOUT_H,TempData);
        ///////////////////////////////// Encoder Test
        //EncoderPoses MyPoses;
        //MyEncoders.GetEncoderPoses(&MyPoses);
        
        ///////////////////////////////// Led Test
/*
          SetRgbLed(true,true,true);
          Delay_ms(10);
          SetRgbLed(false,false,false);
          Delay_ms(500);
*/
    /* Update IWDG counter */
    IWDG_ReloadCounter();
      
    }
}

void Module::Init_WatchDog(void)
{

  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_32);

  /* Set counter reload value to obtain 250ms IWDG TimeOut.
Counter Reload Value = 250ms/IWDG counter clock period
= 250ms / (LSI/32)
= 0.25s / (LsiFreq/32)
= LsiFreq/(32 * 4)
= LsiFreq/128
*/
  IWDG_SetReload(200);

  /* Reload IWDG counter */
  IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();


}

void Module::Delay_ms(int delay)
{
  int i,j;
  for(int j = 0; j < delay; j++)  
    for(int i = 0; i < 8000; i++){}
}
//-----------------------------------------------------------------------------
// Initialises the module and all the attached devices.
//-----------------------------------------------------------------------------

void Module::Initialize()
{
    ConfigurePwm();
    ConfigureLed();
    ConfigureADC();

    MyEncoders.Config();

    m_MPU9150Driver.initialise();

    //initialize the motor driver
    m_MainMotorDriver.Initialize(TIM10,RCC_APB2Periph_TIM10,
                                 TIM11,RCC_APB2Periph_TIM11,
                                 TIM9,RCC_APB2Periph_TIM9,
                                 GPIO_Pin(GPIOD,RCC_AHB1Periph_GPIOD,GPIO_Pin_0),
                                 GPIO_Pin(GPIOD,RCC_AHB1Periph_GPIOD,GPIO_Pin_1),
                                 GPIO_Pin(GPIOB,RCC_AHB1Periph_GPIOB,GPIO_Pin_8,GPIO_PinSource8,GPIO_AF_TIM10),
                                 GPIO_Pin(GPIOD,RCC_AHB1Periph_GPIOD,GPIO_Pin_6),
                                 GPIO_Pin(GPIOD,RCC_AHB1Periph_GPIOD,GPIO_Pin_3),
                                 GPIO_Pin(GPIOB,RCC_AHB1Periph_GPIOB,GPIO_Pin_9,GPIO_PinSource9,GPIO_AF_TIM11),
                                 GPIO_Pin(GPIOD,RCC_AHB1Periph_GPIOD,GPIO_Pin_4),
                                 GPIO_Pin(GPIOD,RCC_AHB1Periph_GPIOD,GPIO_Pin_5),
                                 GPIO_Pin(GPIOE,RCC_AHB1Periph_GPIOE,GPIO_Pin_5,GPIO_PinSource5,GPIO_AF_TIM9));
    m_MainMotorDriver.SetMode(VNH3SP30TRDriver::eModeForward);
    m_MainMotorDriver.SetSpeed(0.0f);
    SetServoPos(0,0.5);
    SetServoPos(1,0.5);
    SetServoPos(2,0.5);
    SetServoPos(3,0.5);

    m_ComsDriver.Initialize(GPIO_Pin(GPIOB,RCC_AHB1Periph_GPIOB,GPIO_Pin_10,GPIO_PinSource10,GPIO_AF_USART3),
                            GPIO_Pin(GPIOB,RCC_AHB1Periph_GPIOB,GPIO_Pin_11,GPIO_PinSource11,GPIO_AF_USART3),
                            GPIO_Pin(GPIOB,RCC_AHB1Periph_GPIOB,GPIO_Pin_13,GPIO_PinSource13,GPIO_AF_USART3),
                            GPIO_Pin(GPIOB,RCC_AHB1Periph_GPIOB,GPIO_Pin_14,GPIO_PinSource14,GPIO_AF_USART3));
    //configure the timer used to measure the dT between
    //consecutive cycles
//                configureTimer();
//		
//		_lsm303.initialise();
//		_imu3000.initialise();
//		_venusGps.initialise();

//		configureInterrupts();
          SetRgbLed(true,true,true);
          Delay_ms(200);
          SetRgbLed(false,false,false);
          Delay_ms(200);
          
          
          Init_WatchDog();
          
}

void Module::ADC_Push2Pack(Transmit_CommandPacket &_data)
{
    _data.ADC_LB = ADC1ConvertedValue[4];
    _data.ADC_LF_yaw = ADC1ConvertedValue[0];
    _data.ADC_LF_rol = ADC1ConvertedValue[1];
    _data.ADC_RB = ADC1ConvertedValue[5];
    _data.ADC_RF_yaw = ADC1ConvertedValue[2];
    _data.ADC_RF_rol = ADC1ConvertedValue[3];
}
void Module::Adc_Comm_Init(void)
{
   ADC_CommonInitTypeDef ADC_CommonInitStructure; 
  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;//ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
}

void Module::Adc_Channel_Init(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;//DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 6;
  ADC_Init(ADC1, &ADC_InitStructure);
  /* ADC1 regular channel configuration *************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 2, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 4, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 5, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 6, ADC_SampleTime_3Cycles);
}
void Module::DMA_Config(void)
{
  DMA_InitTypeDef       DMA_InitStructure;
  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 6;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;        
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);
}

void Module::GPIO_Config(void)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  /* Configure ADC1 Channel10-Channel15 pins as analog input ******************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}
///////////////////////////////////////////////////////////////////////////////
void Module::ConfigureADC()
{
  /* Enable ADC1, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  DMA_Config();
  GPIO_Config();
  Adc_Comm_Init();
  Adc_Channel_Init();
  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_SoftwareStartConv(ADC1);
}

///////////////////////////////////////////////////////////////////////////////
void Module::ConfigurePwm()
{
    //configure the GPIO pins
    RCCSetClock(GPIOD,RCC_AHB1Periph_GPIOD,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    // pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15,GPIO_AF_TIM4);


    RCCSetClock(TIM4,RCC_APB1Periph_TIM4,ENABLE);

    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    //configure the timer
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    // Compute the prescaler value
    //200Khz timer clock, period at 2ms equals [0.01 / (1/2000000)] = 20000 ticks
    //therefore servo positions between 1ms and 2ms equal 2000->4000 ticks (repeated 100 times a second)
    int a = (int)RCC_Clocks.PCLK2_Frequency;
    uint16_t prescalerValue = (uint16_t)((double)RCC_Clocks.PCLK2_Frequency / (double)m_nPwmFreq) - 1;

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = m_nPwmPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock / 2  (Hz), to get TIM3 counter
    clock at 20 MHz the Prescaler is computed as following:
    - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 120 MHz for STM32F2xx devices

    The TIM3 is running at 30 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                          = 20 MHz / 666 = 30 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
    ----------------------------------------------------------------------- */

    TIM_OCInitTypeDef OCInitStructure;
    TIM_OCStructInit(&OCInitStructure);

    /* PWM1 Mode configuration: Channel1 */
    OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    OCInitStructure.TIM_Pulse = m_nPwmMid;
    OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM4, &OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OC2Init(TIM4, &OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OC3Init(TIM4, &OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 */
    TIM_OC3Init(TIM4, &OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);

    /* TIM3 enable counter */
    TIM_Cmd(TIM4, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
void Module::ConfigureLed()
{
    //Enable PIOE clock which controls the LED
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_SetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4);
}

///////////////////////////////////////////////////////////////////////////////
void Module::SetRgbLed(const bool r, const bool g, const bool b)
{
    if(r){
        GPIO_ResetBits(GPIOE,GPIO_Pin_2);
     }else{
        GPIO_SetBits(GPIOE,GPIO_Pin_2);
    }

    if(g){
        GPIO_ResetBits(GPIOE,GPIO_Pin_3);
     }else{
        GPIO_SetBits(GPIOE,GPIO_Pin_3);
    }

    if(b){
        GPIO_ResetBits(GPIOE,GPIO_Pin_4);
     }else{
        GPIO_SetBits(GPIOE,GPIO_Pin_4);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Servo position range is 0-2000
///////////////////////////////////////////////////////////////////////////////
void Module::SetServoPos(const int nServo, const float dPos)
{

    float dClampedPos;
    if(dPos>1.0f)
      dClampedPos = 1.0f;
    else if(dPos<0)
      dClampedPos = 0.0f;
    else
      dClampedPos = dPos;

    int nClampedPos = (dClampedPos * (float)(m_nPwmMax-m_nPwmMin) ) + m_nPwmMin;
    switch(nServo){
        case 0:
            TIM_SetCompare1(TIM4, nClampedPos);
            break;

        case 1:
            TIM_SetCompare2(TIM4, nClampedPos);
            break;

        case 2:
            TIM_SetCompare3(TIM4, nClampedPos);
            break;

        case 3:
            TIM_SetCompare4(TIM4, nClampedPos);
            break;

    }
}

//-----------------------------------------------------------------------------
// Configure the interrupts for the module
//-----------------------------------------------------------------------------
void Module::ConfigureInterrupts()
{
    //register this class to receive interrupts
    InterruptTemplate::registerForInterrupt(TIM2_INTERRUPT_HANDLER,this);
    InterruptTemplate::registerForInterrupt(SYSTICK_INTERRUPT_HANDLER,this);

    NVIC_InitTypeDef NVIC_InitStructure;
    //configure the timer interrupt
    NVIC_InitStructure.NVIC_IRQChannel = MODULE_TIMER_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //and now enable the Update (overflow) interrupt bit
    TIM_ITConfig(MODULE_TIMER,TIM_IT_Update, ENABLE);

}


//-----------------------------------------------------------------------------
// Systick interrupt handlers, synchronises the periodic tasks of the module
//-----------------------------------------------------------------------------
void Module::onInterruptSystick()
{


}

