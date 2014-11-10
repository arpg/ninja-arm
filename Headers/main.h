#pragma once
#define MAXSIZE 7

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx.h"
#include <libarm.h>
#include <stdio.h>
#include <string.h>
#include "mathRoutines.h"
#include "fixed.h"

/* Macros ------------------------------------------------------------------*/
#define int16LSB(x) x & 0xFF
#define int16MSB(x) (x>>8) & 0xFF

/* Defines ------------------------------------------------------------------*/
#define NULL 0

extern __IO uint32_t TimingDelay;
extern __IO float SysTickDelayMs;

#define setTimeout(x) TimingDelay = x
#define isTimedOut() TimingDelay <= 0
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

unsigned long getTicks();
float getTimeSpan(unsigned long tickEnd, unsigned long tickStart);
void fail();

#define ASSERT_FAIL(x) if( x == false ){ fail(); }

extern "C" void __cxa_pure_virtual(void);
