#include "ShaftEncoder.h"
//#include "STM32vldiscovery.h"

void Encoder::Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructurePan;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructurePan;
  
  //Encoder 1
  // clock for GPIO
  RCC_AHB1PeriphClockCmd(Encoder1_RCC_AHB1Periph_GPIO, ENABLE);
  // GPIO Initialize
  GPIO_InitStructurePan.GPIO_Pin = Encoder1_CHA_GPIO_Pin | Encoder1_CHB_GPIO_Pin;
  GPIO_InitStructurePan.GPIO_PuPd = Encoder1_GPIO_PuPd;
  GPIO_InitStructurePan.GPIO_Mode = Encoder1_GPIO_MODE;
  GPIO_Init(Encoder1_GPIO_PORT,&GPIO_InitStructurePan);
  GPIO_PinAFConfig(Encoder1_GPIO_PORT, Encoder1_SourceA, Encoder1_GPIO_AF_TIM);
  GPIO_PinAFConfig(Encoder1_GPIO_PORT, Encoder1_SourceB, Encoder1_GPIO_AF_TIM);
  Encoder1_Timer_Clock_Enable;
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructurePan); 
  TIM_TimeBaseStructurePan.TIM_Period = Encoder1_TIMPeriod;
  TIM_TimeBaseStructurePan.TIM_Prescaler = 0;
  TIM_TimeBaseStructurePan.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(Encoder1_TIMER, &TIM_TimeBaseStructurePan);
  
  TIM_EncoderInterfaceConfig(Encoder1_TIMER, Encoder1_TIM_EncoderMode, Encoder1_TIM_IC1Polarity, Encoder1_TIM_IC2Polarity);
  TIM_SetAutoreload (Encoder1_TIMER, Encoder1_AutoReloadVal); 
  Encoder1_TIMER->CNT = 0;
  TIM_Cmd(Encoder1_TIMER, ENABLE);
    
  //Encoder 2
  // clock for GPIO
  RCC_AHB1PeriphClockCmd(Encoder2_RCC_AHB1Periph_GPIO, ENABLE);
  // GPIO Initialize
  GPIO_InitStructurePan.GPIO_Pin = Encoder2_CHA_GPIO_Pin | Encoder2_CHB_GPIO_Pin;
  GPIO_InitStructurePan.GPIO_PuPd = Encoder2_GPIO_PuPd;
  GPIO_InitStructurePan.GPIO_Mode = Encoder2_GPIO_MODE;
  GPIO_Init(Encoder2_GPIO_PORT,&GPIO_InitStructurePan);
  GPIO_PinAFConfig(Encoder2_GPIO_PORT, Encoder2_SourceA, Encoder2_GPIO_AF_TIM);
  GPIO_PinAFConfig(Encoder2_GPIO_PORT, Encoder2_SourceB, Encoder2_GPIO_AF_TIM);
  Encoder2_Timer_Clock_Enable;
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructurePan); 
  TIM_TimeBaseStructurePan.TIM_Period = Encoder2_TIMPeriod;
  TIM_TimeBaseStructurePan.TIM_Prescaler = 0;
  TIM_TimeBaseStructurePan.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(Encoder2_TIMER, &TIM_TimeBaseStructurePan);
  
  TIM_EncoderInterfaceConfig(Encoder2_TIMER, Encoder2_TIM_EncoderMode, Encoder2_TIM_IC1Polarity, Encoder2_TIM_IC2Polarity);
  TIM_SetAutoreload (Encoder2_TIMER, Encoder2_AutoReloadVal); 
  Encoder2_TIMER->CNT = 0;
  TIM_Cmd(Encoder2_TIMER, ENABLE);

  //Encoder 3
  // clock for GPIO
  RCC_AHB1PeriphClockCmd(Encoder3_RCC_AHB1Periph_GPIO, ENABLE);
  // GPIO Initialize
  GPIO_InitStructurePan.GPIO_Pin = Encoder3_CHA_GPIO_Pin | Encoder3_CHB_GPIO_Pin;
  GPIO_InitStructurePan.GPIO_PuPd = Encoder3_GPIO_PuPd;
  GPIO_InitStructurePan.GPIO_Mode = Encoder3_GPIO_MODE;
  GPIO_Init(Encoder3_GPIO_PORT,&GPIO_InitStructurePan);
  GPIO_PinAFConfig(Encoder3_GPIO_PORT, Encoder3_SourceA, Encoder3_GPIO_AF_TIM);
  GPIO_PinAFConfig(Encoder3_GPIO_PORT, Encoder3_SourceB, Encoder3_GPIO_AF_TIM);
  Encoder3_Timer_Clock_Enable;
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructurePan); 
  TIM_TimeBaseStructurePan.TIM_Period = Encoder3_TIMPeriod;
  TIM_TimeBaseStructurePan.TIM_Prescaler = 0;
  TIM_TimeBaseStructurePan.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(Encoder3_TIMER, &TIM_TimeBaseStructurePan);
  
  TIM_EncoderInterfaceConfig(Encoder3_TIMER, Encoder3_TIM_EncoderMode, Encoder3_TIM_IC1Polarity, Encoder3_TIM_IC2Polarity);
  TIM_SetAutoreload (Encoder3_TIMER, Encoder3_AutoReloadVal); 
  Encoder3_TIMER->CNT = 0;
  TIM_Cmd(Encoder3_TIMER, ENABLE);

  //Encoder 4
  // clock for GPIO
  RCC_AHB1PeriphClockCmd(Encoder4_RCC_AHB1Periph_GPIO, ENABLE);
  // GPIO Initialize
  GPIO_InitStructurePan.GPIO_Pin = Encoder4_CHA_GPIO_Pin | Encoder4_CHB_GPIO_Pin;
  GPIO_InitStructurePan.GPIO_PuPd = Encoder4_GPIO_PuPd;
  GPIO_InitStructurePan.GPIO_Mode = Encoder4_GPIO_MODE;
  GPIO_Init(Encoder4_GPIO_PORT,&GPIO_InitStructurePan);
  GPIO_PinAFConfig(Encoder4_GPIO_PORT, Encoder4_SourceA, Encoder4_GPIO_AF_TIM);
  GPIO_PinAFConfig(Encoder4_GPIO_PORT, Encoder4_SourceB, Encoder4_GPIO_AF_TIM);
  Encoder4_Timer_Clock_Enable;
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructurePan); 
  TIM_TimeBaseStructurePan.TIM_Period = Encoder4_TIMPeriod;
  TIM_TimeBaseStructurePan.TIM_Prescaler = 0;
  TIM_TimeBaseStructurePan.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(Encoder4_TIMER, &TIM_TimeBaseStructurePan);
  
  TIM_EncoderInterfaceConfig(Encoder4_TIMER, Encoder4_TIM_EncoderMode, Encoder4_TIM_IC1Polarity, Encoder4_TIM_IC2Polarity);
  TIM_SetAutoreload (Encoder4_TIMER, Encoder4_AutoReloadVal); 
  Encoder4_TIMER->CNT = 0;
  TIM_Cmd(Encoder4_TIMER, ENABLE);
}

void Encoder::GetEncoderPoses(EncoderPoses& Position)
{
  Position.LB = Encoder2_TIMER->CNT;
//  Encoder2_TIMER->CNT = 0;
  Position.LF = Encoder4_TIMER->CNT;
//  Encoder4_TIMER->CNT = 0;
  Position.RB = Encoder1_TIMER->CNT;
//  Encoder1_TIMER->CNT = 0;
  Position.RF = Encoder3_TIMER->CNT;
//  Encoder3_TIMER->CNT = 0;
}

void Encoder::Push2Pack(Transmit_CommandPacket &_data)
{
  EncoderPoses  Pos;
  GetEncoderPoses(Pos);
  _data.Enc_LB = Pos.LB;
  _data.Enc_LF = Pos.LF;
  _data.Enc_RB = Pos.RB;
  _data.Enc_RF = Pos.RF;
}