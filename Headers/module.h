//-----------------------------------------------------------------------------
// module.h
//
// This class contains the majority of the functionality driving the IMU module.
// It will automate all functionality of the module, initialize the peripherals
// and sensors and relay information back to the GS.
// Author: Nima Keivan
// Revision: Sina Aghli
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer settings for the module
//-----------------------------------------------------------------------------
#define MODULE_TIMER_CLOCK			15000000					//66.66nS resolution
#define MODULE_TIMER_PERIOD			0xFFFFFFFF					//overflow every 4 minutes @15mhZ
#define MODULE_TIMER				TIM2						//32 bit timer
#define MODULE_TIMER_CLK			RCC_APB1Periph_TIM2	
#define MODULE_TIMER_IRQ			TIM2_IRQn	
#define MODULE_TICKS_TO_SECONDS                 0.0000000666


//-----------------------------------------------------------------------------
// network Settings
//-----------------------------------------------------------------------------
#define IKNETWORK_COORD_ADDRESS		0x0000	//address of the coordinator 16 bit
#define IKNETWORK_STATUS_POLL_TIME	500	//twice a second poll
#define IKNETWORK_JOIN_TIME			500	//try to join twice a second

#include "Utilities/InterruptTemplate.h"
#include "Drivers/VNH3SP30TRdriver.h"
#include "Drivers/ftdicomsdriver.h"  
#include "Headers/ShaftEncoder.h"
#include "Drivers/USART3DMA.h"
//#include "Drivers/MPU9150driverDMA.h"
#include "Drivers/MPU9150driver.h"

class Module : InterruptTemplate
{
    private:


      void configureTimer();
      void ConfigureLed();
      void ConfigureADC();
      void ConfigurePwm();
      void ConfigurePwm_test();
      void ConfigureInterrupts();
      void Adc_Comm_Init();
      void Adc_Channel_Init();
      void DMA_Config();
      void GPIO_Config();
      void ADC_Push2Pack(Transmit_CommandPacket &_data);
      void Init_WatchDog();
      void TIM3_Configuration();
      virtual void onInterruptSystick();

      static uint16_t m_nPwmPeriod;
      static uint16_t m_nPwmMin;
      static uint16_t m_nPwmMax;
      static uint32_t m_nPwmFreq;
      static uint16_t m_nPwmMid;
      static uint16_t acc_MaxRange;
      static uint16_t acc_MinRange;

    public:
      //VNH3SP30TRDriver m_MainMotorDriver;
      FtdiComsDriver          m_ComsDriver;
      Encoder                 MyEncoders;
      MPU9150Driver           m_MPU9150Driver;
      MPU9150_Data_Structure  IMU_Data;
      serialdriver            usart3driver;
      void Initialize();
      Module();
      void Run();
      void SetRgbLed(const bool r, const bool g);
      void SetServoPos(const int nServo, const float dPos);
      void ConfigureBLDC();
      void Delay_ms(int delay);
      float temp_float[4]={0,0,0,0};
};

