/**
  ******************************************************************************
  * @file    Project/STM32F2xx_StdPeriph_Template/main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"
#include "stm322xg_eval.h"
#include "stm322xg_eval_lcd.h"
#include <stdio.h>
#include <libarm.h>
#include <string.h>
#include "mathRoutines.h"
#include "fixed.h"

#define MAXSIZE 7

void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/*
 
#ifndef __MAIN_H
#define __MAIN_H

#define MAXSIZE 7

#include <libarm.h>
#include <string.h>
//#include "mathRoutines.h"
//#include "fixed.h"

#include "stm32f2xx.h"
#include "stm322xg_eval.h"
#include "stm322xg_eval_lcd.h"
#include <stdio.h>

void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

#define int16LSB(x) x & 0xFF
#define int16MSB(x) (x>>8) & 0xFF


#define NULL 0
#define ASSERT_FAIL(x) if( x == false ){ fail(); }
#define setTimeout(x) TimingDelay = x
#define isTimedOut() TimingDelay <= 0

//extern __IO uint32_t TimingDelay;
extern __IO float SysTickDelayMs;

unsigned long getTicks();

float getTimeSpan(unsigned long tickEnd, unsigned long tickStart);

void fail();

//extern "C" void __cxa_pure_virtual(void);

#endif 


*/