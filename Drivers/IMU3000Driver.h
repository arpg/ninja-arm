//-----------------------------------------------------------------------------
// IMU3000Driver.h
//
// This file contains the implementation of the I2C driver to communicate with
// the IMU3000 sensor
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// The I2C port definitions used for the communciation
//-----------------------------------------------------------------------------
#define IMU3000_I2Cx                          I2C2
#define IMU3000_I2Cx_IRQ				      I2C2_EV_IRQn
#define IMU3000_I2Cx_CLK                      RCC_APB1Periph_I2C2
#define IMU3000_I2Cx_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define IMU3000_I2Cx_SDA_PIN                  GPIO_Pin_11                
#define IMU3000_I2Cx_SDA_GPIO_PORT            GPIOB                       
#define IMU3000_I2Cx_SDA_SOURCE               GPIO_PinSource11
#define IMU3000_I2Cx_AF					      GPIO_AF_I2C2
  
#define IMU3000_I2Cx_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define IMU3000_I2Cx_SCL_PIN                  GPIO_Pin_10                
#define IMU3000_I2Cx_SCL_GPIO_PORT            GPIOB                    
#define IMU3000_I2Cx_SCL_SOURCE               GPIO_PinSource10

#define IMU3000_I2Cx_DMA                      DMA1
#define IMU3000_I2Cx_DMA_CHANNEL              DMA_Channel_7
#define IMU3000_I2Cx_DR_ADDRESS               &IMU3000_I2Cx->DR		//this is the address of the register that the DMA uses
#define IMU3000_I2Cx_DMA_STREAM_RX            DMA1_Stream2
#define IMU3000_I2Cx_DMA_STREAM_RX_IRQ        DMA1_Stream2_IRQn

#define IMU3000_I2Cx_TX_DMA_TCFLAG            DMA_FLAG_TCIF7
#define IMU3000_I2Cx_TX_DMA_FEIFLAG           DMA_FLAG_FEIF7
#define IMU3000_I2Cx_TX_DMA_DMEIFLAG          DMA_FLAG_DMEIF7
#define IMU3000_I2Cx_TX_DMA_TEIFLAG           DMA_FLAG_TEIF7
#define IMU3000_I2Cx_TX_DMA_HTIFLAG           DMA_FLAG_HTIF7
#define IMU3000_I2Cx_RX_DMA_TCFLAG            DMA_FLAG_TCIF2
#define IMU3000_I2Cx_RX_DMA_FEIFLAG           DMA_FLAG_FEIF2
#define IMU3000_I2Cx_RX_DMA_DMEIFLAG          DMA_FLAG_DMEIF2
#define IMU3000_I2Cx_RX_DMA_TEIFLAG           DMA_FLAG_TEIF2
#define IMU3000_I2Cx_RX_DMA_HTIFLAG           DMA_FLAG_HTIF2
#define IMU3000_DMAx_CLK                      RCC_AHB1Periph_DMA1    

//-----------------------------------------------------------------------------
// GPIO port definitions for the DRDY interrupt
//-----------------------------------------------------------------------------
#define IMU3000_DRDY_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define IMU3000_DRDY_PIN					GPIO_Pin_5                
#define IMU3000_DRDY_GPIO_PORT				GPIOA         
#define IMU3000_DRDY_PIN_EXT_PORT_SOURCE	EXTI_PortSourceGPIOA
#define IMU3000_DRDY_PIN_EXT_PIN_SOURCE		EXTI_PinSource5
#define IMU3000_DRDY_PIN_EXT_PIN_LINE		EXTI_Line5
#define IMU3000_DRDY_PIN_EXT_PIN_IRQ		EXTI9_5_IRQn
#define IMU3000_DRDY_PIN_EXT_PIN_HANDLER	EXTI5_INTERRUPT_HANDLER

//-----------------------------------------------------------------------------
// I2C can operate either in fast mode or standard mode
//-----------------------------------------------------------------------------
#define IMU3000_FAST_I2C_MODE

#ifdef IMU3000_FAST_I2C_MODE
 #define IMU3000_I2C_SPEED 340000
 #define IMU3000_I2C_DUTYCYCLE I2C_DutyCycle_16_9  
#else /* STANDARD_I2C_MODE*/
 #define IMU3000_I2C_SPEED 100000
 #define IMU3000_I2C_DUTYCYCLE  I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/

//-----------------------------------------------------------------------------
// Size of RX TX buffers
//-----------------------------------------------------------------------------
#define IMU3000_I2C_RX_BUFFER_SIZE			  24
#define IMU3000_DMA_READ_LENGTH				  8

//-----------------------------------------------------------------------------
// IMU3000 settings
//----------------------------------------------------------------------------	-
#define IMU3000_SENSITIVITY					IMU3000_FS_SEL_1000
#define IMU3000_SENSITIVITY_LSB				0.001	//this is in degrees per second

//-----------------------------------------------------------------------------
// Address of the IMU3000 itself
//-----------------------------------------------------------------------------
#define IMU3000_ADDRESS 0xD0// b11010000

//-----------------------------------------------------------------------------
// Structure defining the readout from the IMU3000
//-----------------------------------------------------------------------------
struct IMU3000_Data_Structure
{
	signed short _temperature;
	signed short _gyroX;
    signed short _gyroY;
    signed short _gyroZ;
};


namespace Andromeda
{
	class IMU3000Driver : InterruptTemplate
	{
		public:
		void initialise();
        bool getRegister(char address, char &value);
		void setRegister(char address, char value);
		bool getData(float &xRate, float &yRate, float &zRate);

		private:
		void configureI2c();
		void configureDevice();
		void configureInterrupts();

        virtual void onInterruptExti5();
        virtual void onInterruptDma1Stream2();
        virtual void onInterruptI2c2Ev();

		char _rxBuffer[IMU3000_I2C_RX_BUFFER_SIZE];
		bool _registerAddressSent;
		bool _dataUpdated;
        IMU3000_Data_Structure _data;

        EXTI_InitTypeDef   _extInitStructure;
	};

}