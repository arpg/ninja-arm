//-----------------------------------------------------------------------------
// LSM303Driver.h
//
// This file contains the implementation of the I2C driver to communicate with
// the LSM303DLH sensor
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------
#include "stm32f2xx_exti.h"
//-----------------------------------------------------------------------------
// The I2C port definitions used for the communciation
//-----------------------------------------------------------------------------
#define LSM303_I2Cx                          I2C1
#define LSM303_I2Cx_IRQ                       I2C1_EV_IRQn
#define LSM303_I2Cx_CLK                      RCC_APB1Periph_I2C1
#define LSM303_I2Cx_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define LSM303_I2Cx_SDA_PIN                  GPIO_Pin_7                
#define LSM303_I2Cx_SDA_GPIO_PORT            GPIOB                       
#define LSM303_I2Cx_SDA_SOURCE               GPIO_PinSource7
#define LSM303_I2Cx_SDA_AF                   GPIO_AF_I2C2
  
#define LSM303_I2Cx_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define LSM303_I2Cx_SCL_PIN                  GPIO_Pin_6            
#define LSM303_I2Cx_SCL_GPIO_PORT            GPIOB                    
#define LSM303_I2Cx_SCL_SOURCE               GPIO_PinSource6
#define LSM303_I2Cx_SCL_AF                   GPIO_AF_I2C2

#define LSM303_I2Cx_DMA                      DMA1
#define LSM303_I2Cx_DMA_CHANNEL              DMA_Channel_1
#define LSM303_I2Cx_DR_ADDRESS               &LSM303_I2Cx->DR		//this is the address of the register that the DMA uses
#define LSM303_I2Cx_DMA_STREAM_RX            DMA1_Stream0
#define LSM303_I2Cx_DMA_STREAM_RX_IRQ        DMA1_Stream0_IRQn

#define LSM303_I2Cx_RX_DMA_TCFLAG            DMA_FLAG_TCIF0
#define LSM303_I2Cx_RX_DMA_FEIFLAG           DMA_FLAG_FEIF0
#define LSM303_I2Cx_RX_DMA_DMEIFLAG          DMA_FLAG_DMEIF0
#define LSM303_I2Cx_RX_DMA_TEIFLAG           DMA_FLAG_TEIF0
#define LSM303_I2Cx_RX_DMA_HTIFLAG           DMA_FLAG_HTIF0
#define LSM303_DMAx_CLK                      RCC_AHB1Periph_DMA1    

//-----------------------------------------------------------------------------
// GPIO port definitions for the DRDY interrupt for the accelerometer (this will
// be in INT2)
//-----------------------------------------------------------------------------
#define LSM303_DRDY_A_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define LSM303_DRDY_A_PIN					GPIO_Pin_1                
#define LSM303_DRDY_A_GPIO_PORT				GPIOA         
#define LSM303_DRDY_A_PIN_EXT_PORT_SOURCE	EXTI_PortSourceGPIOA
#define LSM303_DRDY_A_PIN_EXT_PIN_SOURCE	EXTI_PinSource1
#define LSM303_DRDY_A_PIN_EXT_PIN_LINE		EXTI_Line1
#define LSM303_DRDY_A_PIN_EXT_PIN_IRQ		EXTI1_IRQn
#define LSM303_DRDY_A_PIN_EXT_PIN_HANDLER	EXTI1_INTERRUPT_HANDLER

//-----------------------------------------------------------------------------
// GPIO port definitions for the DRDY interrupt for the magnetometer
//-----------------------------------------------------------------------------
#define LSM303_DRDY_M_GPIO_CLK				RCC_AHB1Periph_GPIOA
#define LSM303_DRDY_M_PIN					GPIO_Pin_4                
#define LSM303_DRDY_M_GPIO_PORT				GPIOA         
#define LSM303_DRDY_M_PIN_EXT_PORT_SOURCE	EXTI_PortSourceGPIOA
#define LSM303_DRDY_M_PIN_EXT_PIN_SOURCE	EXTI_PinSource4
#define LSM303_DRDY_M_PIN_EXT_PIN_LINE		EXTI_Line4
#define LSM303_DRDY_M_PIN_EXT_PIN_IRQ		EXTI4_IRQn
#define LSM303_DRDY_M_PIN_EXT_PIN_HANDLER	EXTI4_INTERRUPT_HANDLER

//-----------------------------------------------------------------------------
// I2C can operate either in fast mode or standard mode
//-----------------------------------------------------------------------------
#define LSM303_FAST_I2C_MODE

#ifdef LSM303_FAST_I2C_MODE
 #define LSM303_I2C_SPEED 340000
 #define LSM303_I2C_DUTYCYCLE I2C_DutyCycle_16_9  
#else /* STANDARD_I2C_MODE*/
 #define LSM303_I2C_SPEED 100000
 #define LSM303_I2C_DUTYCYCLE  I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/

//-----------------------------------------------------------------------------
// Size of RX TX buffers
//-----------------------------------------------------------------------------
#define LSM303_I2C_RX_BUFFER_SIZE			  24
#define LSM303_A_DMA_READ_LENGTH			  6

//-----------------------------------------------------------------------------
// LSM303 settings
//----------------------------------------------------------------------------	-
#define LSM303_MAGNETOMETER					1
#define LSM303_ACCELEROMETER				2
#define LSM303_NONE							0
#define LSM_ACCEL_SENSITIVITY				LSM303_A_FS_2G
#define LSM_ACCEL_SENSITIVITY_LSB			0.001	//this is in Gs per LSB
#define LSM_MAG_SENSITIVITY					LSM303_M_1_3GAUSS
#define LSM_MAG_SENSITIVITY_LSB_XY			0.000947867299	//this is in gauss per LSB for XY axes
#define LSM_MAG_SENSITIVITY_LSB_Z			0.00105263158   //this is in gauss per LSB for Z axis
//-----------------------------------------------------------------------------
// Address of the LSM303 itself for acceleration and magnetometer
//-----------------------------------------------------------------------------
#define LSM303_ADDRESS_A 0x30// b00110001
#define LSM303_ADDRESS_M 0x3C// b00111100

//-----------------------------------------------------------------------------
// Structure defining the readout from the LSM303 accelerometer
//-----------------------------------------------------------------------------
struct __attribute__ ((__packed__)) LSM303_Accel_Data_Structure
{
	signed short _accelX;
	signed short _accelY;
    signed short _accelZ;
};

//-----------------------------------------------------------------------------
// Structure defining the readout from the LSM303 magnetometer
//-----------------------------------------------------------------------------
struct __attribute__ ((__packed__)) LSM303_Mag_Data_Structure
{
	signed short _magX;
	signed short _magY;
    signed short _magZ;
};

namespace Andromeda
{
	class LSM303Driver : InterruptTemplate
	{
		public:
		void initialise();
        bool getRegister(char deviceAddress, char address, char &value);
		void setRegister(char deviceAddress, char address, char value);
        bool getMagData(float &xMag, float &yMag, float &zMag);
        bool getAccelData(float &xAccel, float &yAccel, float &zAccel);

		bool hasAccelData() { return _accelDataUpdated; }
        bool hasMagData() { return _magDataUpdated; }

		private:
		void configureI2c();
		void configureDevice();
		void configureInterrupts();

		void startAccelerometerDma();
        void startMagnetometerDma();

        virtual void onInterruptExti1();
        virtual void onInterruptExti4();
        virtual void onInterruptI2c1Ev();
		virtual void onInterruptDma1Stream0();

        EXTI_InitTypeDef   _extInitStructure;
		bool _registerAddressSent;
		int _currentDmaOperation;
		int _nextDmaOperation;

        bool _magDataUpdated;
        bool _accelDataUpdated;
        LSM303_Accel_Data_Structure _accelData;
        LSM303_Mag_Data_Structure _magData;

		char _rxBuffer[LSM303_I2C_RX_BUFFER_SIZE];
	};

}