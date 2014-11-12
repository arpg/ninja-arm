#include "Utils.h"

///////////////////////////////////////////////////////////////////////////////
int RCCSetClock(const void* pPeriph, const uint32_t nClockId, const FunctionalState NewState)
{

    uint32_t nPeriph = (uint32_t)pPeriph;
    if(nPeriph >= APB1PERIPH_BASE && nPeriph < APB2PERIPH_BASE ){
        RCC_APB1PeriphClockCmd(nClockId,NewState);
    }else if(nPeriph >= APB2PERIPH_BASE && nPeriph <= AHB1PERIPH_BASE){
        RCC_APB2PeriphClockCmd(nClockId,NewState);
    }else if(nPeriph >= AHB1PERIPH_BASE && nPeriph <= AHB2PERIPH_BASE){
        RCC_AHB1PeriphClockCmd(nClockId,NewState);
    }else if(nPeriph >= AHB2PERIPH_BASE){
        RCC_AHB2PeriphClockCmd(nClockId,NewState);
    }else if(nClockId == RCC_AHB3Periph_FSMC){
        RCC_AHB3PeriphClockCmd(nClockId,NewState);
    }

    int b=0;
    for(int ii=0;ii<10;ii++)
      b++;
    return(b);
}

