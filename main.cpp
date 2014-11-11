/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "imuKalman.h"
#include "interruptTemplate.h"
#include "UartPacketDriver.h"
//#include "MPU9150Driver.h"
//#include "MPU9150DriverDMA.h"
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_gpio.h"
//#include "LSM303Driver.h"
//#include "IMU3000Driver.h"
//#include "VenusGpsDriver.h"
#include "module.h"
//#include "interruptTemplate.h"

unsigned long _totalTicks = 0;

 void __cxa_pure_virtual(void)
{
// call to a pure virtual function happened ... wow, should never happen ... stop
while(1);
}


using namespace Andromeda;
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t TimingDelay;
__IO float SysTickDelayMs;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  RCC_ClocksTypeDef RCC_Clocks;

  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f2xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f2xx.c file
     */  

  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

  int aa = 0;
  for(int i=0;i<3;i++)
    aa++;


 // SystemCoreClockUpdate();
   // int i = SystemCoreClock; 
    int b=0;
    for( int j = 0; j < 5 ; j++ )
    {
      b++;
    }
    //reset all interrupt handlers
    InterruptTemplate::initialiseHandlers();

    /* SysTick end of count event each 10ms */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTickDelayMs = 10;
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

    //MPU9150Driver driver;
    //driver.initialise();

    Module module;
    module.Delay_ms(500);    // wait for voltage to become stable
    module.Initialize();
    module.Run();



}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

//-----------------------------------------------------------------------------
// Returns the current elapsed tick count
//-----------------------------------------------------------------------------
//unsigned long getTicks()
//{
//	disableInterrupts();
//	if( TIM_GetFlagStatus(MODULE_TIMER,TIM_IT_Update) == ENABLE )
//	{
//		//then an overflow interrupt is pending, we must tend to it now
//		_totalTicks += MODULE_TIMER_PERIOD;
//	}
//
//	unsigned long ticks = _totalTicks +TIM_GetCounter(MODULE_TIMER);
//	enableInterrupts();
//	return ticks;
//}

//-----------------------------------------------------------------------------
//	Based on the given ticks, returns time in seconds
//-----------------------------------------------------------------------------
//float getTimeSpan(unsigned long tickEnd, unsigned long tickStart)
//{
//	unsigned long totalTicks =0 ;
//	if( tickStart > tickEnd )
//	{
//		if( (tickStart - tickEnd) < TIM_GetCounter(MODULE_TIMER) )
//		{
//			//this is an error state where the TC has rolled over but the interrupt to 
//			//service it has not done so yet and the value hasn't been added to _totalTicks
//			return 0;
//		}
//
//		//then counts have rolled over
//		totalTicks = tickEnd + (MODULE_TIMER_PERIOD - tickStart);
//	}
//	else
//		totalTicks = tickEnd - tickStart;
//
//	//return the time in seconds
//	return totalTicks *  MODULE_TICKS_TO_SECONDS;
//}

//-----------------------------------------------------------------------------
// The fail loop. 
//-----------------------------------------------------------------------------
void fail()
{
	while(1)
	{
	}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
