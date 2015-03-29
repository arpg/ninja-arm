//-----------------------------------------------------------------------------
// MPU9150Driver.h
//
// This file contains the implementation of the I2C driver to communicate with
// the MPU9150 sensor
//
// Author: Nima Keivan, Sina Aghli
//-----------------------------------------------------------------------------

#include "stm32f2xx_exti.h"
#include "USART3DMA.h"
//-----------------------------------------------------------------------------
// The I2C port definitions used for the communciation
//-----------------------------------------------------------------------------
#define MPU9150_I2Cx                          I2C3
#define MPU9150_I2Cx_IRQ		      I2C3_EV_IRQn
#define MPU9150_I2Cx_CLK                      RCC_APB1Periph_I2C3
#define MPU9150_I2Cx_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOC //pins PA8 and PC9
#define MPU9150_I2Cx_SDA_PIN                  GPIO_Pin_9
#define MPU9150_I2Cx_SDA_GPIO_PORT            GPIOC
#define MPU9150_I2Cx_SDA_SOURCE               GPIO_PinSource9
#define MPU9150_I2Cx_AF                       GPIO_AF_I2C3

#define MPU9150_I2Cx_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define MPU9150_I2Cx_SCL_PIN                  GPIO_Pin_8
#define MPU9150_I2Cx_SCL_GPIO_PORT            GPIOA
#define MPU9150_I2Cx_SCL_SOURCE               GPIO_PinSource8

#define MPU9150_I2Cx_DMA                      DMA1
#define MPU9150_I2Cx_DMA_CHANNEL              DMA_Channel_3
#define MPU9150_I2Cx_DR_ADDRESS               ((uint32_t)0x40005C10)		//this is the address of the register that the DMA uses
#define MPU9150_I2Cx_DMA_STREAM_RX            DMA1_Stream2
#define MPU9150_I2Cx_DMA_STREAM_RX_IRQ        DMA1_Stream2_IRQn

#define MPU9150_I2Cx_TX_DMA_TCFLAG            DMA_FLAG_TCIF3  
#define MPU9150_I2Cx_TX_DMA_FEIFLAG           DMA_FLAG_FEIF3  
#define MPU9150_I2Cx_TX_DMA_DMEIFLAG          DMA_FLAG_DMEIF3
#define MPU9150_I2Cx_TX_DMA_TEIFLAG           DMA_FLAG_TEIF3 
#define MPU9150_I2Cx_TX_DMA_HTIFLAG           DMA_FLAG_HTIF3 
#define MPU9150_I2Cx_RX_DMA_TCFLAG            DMA_FLAG_TCIF2 
#define MPU9150_I2Cx_RX_DMA_FEIFLAG           DMA_FLAG_FEIF2 
#define MPU9150_I2Cx_RX_DMA_DMEIFLAG          DMA_FLAG_DMEIF2
#define MPU9150_I2Cx_RX_DMA_TEIFLAG           DMA_FLAG_TEIF2  
#define MPU9150_I2Cx_RX_DMA_HTIFLAG           DMA_FLAG_HTIF2  
/*
#define MPU9150_I2Cx_TX_DMA_TCFLAG            0x08000000 
#define MPU9150_I2Cx_TX_DMA_FEIFLAG           0x00400000
#define MPU9150_I2Cx_TX_DMA_DMEIFLAG          0x01000000
#define MPU9150_I2Cx_TX_DMA_TEIFLAG           0x02000000
#define MPU9150_I2Cx_TX_DMA_HTIFLAG           0x04000000
#define MPU9150_I2Cx_RX_DMA_TCFLAG            0x00200000
#define MPU9150_I2Cx_RX_DMA_FEIFLAG           0x00010000
#define MPU9150_I2Cx_RX_DMA_DMEIFLAG          0x00040000
#define MPU9150_I2Cx_RX_DMA_TEIFLAG           0x00080000
#define MPU9150_I2Cx_RX_DMA_HTIFLAG           0x00100000
*/
#define MPU9150_DMAx_CLK                      RCC_AHB1Periph_DMA1

//-----------------------------------------------------------------------------
// GPIO port definitions for the DRDY interrupt
//-----------------------------------------------------------------------------
#define MPU9150_DRDY_GPIO_CLK                   RCC_AHB1Periph_GPIOE
#define MPU9150_DRDY_PIN			GPIO_Pin_0
#define MPU9150_DRDY_GPIO_PORT			GPIOE
#define MPU9150_DRDY_PIN_EXT_PORT_SOURCE	EXTI_PortSourceGPIOE
#define MPU9150_DRDY_PIN_EXT_PIN_SOURCE		EXTI_PinSource0
#define MPU9150_DRDY_PIN_EXT_PIN_LINE		EXTI_Line0
#define MPU9150_DRDY_PIN_EXT_PIN_IRQ		EXTI0_IRQn
#define MPU9150_DRDY_PIN_EXT_PIN_HANDLER	EXTI0_INTERRUPT_HANDLER

//-----------------------------------------------------------------------------
// I2C can operate either in fast mode or standard mode
//-----------------------------------------------------------------------------
#define STANDARD_I2C_MODE   //MPU9150_FAST_I2C_MODE

#ifdef MPU9150_FAST_I2C_MODE
#define MPU9150_I2C_SPEED       340000
#define MPU9150_I2C_DUTYCYCLE I2C_DutyCycle_16_9
#else /* STANDARD_I2C_MODE*/
#define MPU9150_I2C_SPEED       1000000
#define MPU9150_I2C_DUTYCYCLE   I2C_DutyCycle_2
#endif /* FAST_I2C_MODE*/

//-----------------------------------------------------------------------------
// Size of RX TX buffers
//-----------------------------------------------------------------------------
#define MPU9150_I2C_RX_BUFFER_SIZE			  14
#define MPU9150_DMA_READ_LENGTH				  14

//-----------------------------------------------------------------------------
// MPU9150 settings
//----------------------------------------------------------------------------	-
#define MPU9150_SENSITIVITY				MPU9150_FS_SEL_1000
#define MPU9150_SENSITIVITY_LSB				0.001	//this is in degrees per second

//-----------------------------------------------------------------------------
// Address of the MPU9150 itself
//-----------------------------------------------------------------------------
//ATTENTION
// the 10-bit mode, the address is written to bits [0 .. 9], and in 7-bit - in bits [1 .. 7]
// For Example for slave address 0b1101000 (0x68) you should enter 0x11010000 (0xD0)
#define MPU9150_ADDRESS         0x68<<1  // b1101000
#define MPU9150_RA_MAG_ADDRESS  0x0C<<1
//-----------------------------------------------------------------------------
// The I2C registers
//-----------------------------------------------------------------------------
#define MPU9150_ACCEL_XOUT_H  0x3B
#define MPU9150_PWR_MGMT_1    0x6B
#define MPU9150_Int_Pin_CFG   0x37
#define MPU9150_Ext_IRQen     0x38
//-----------------------------------------------------------------------------
// Structure defining the readout from the MPU9150
//-----------------------------------------------------------------------------
struct MPU9150_Data_Structure
{
    signed short _gyroZ;
    signed short _gyroY;
    signed short _gyroX;
    signed short _temperature;
    signed short _accZ;
    signed short _accY;
    signed short _accX;
    
};


class MPU9150Driver : InterruptTemplate
{
public:
    void initialise();
    bool getRegister(char DesAddress, char RegAddress, char &value);
    void setRegister(char DesAddress, char RegAddress, char value);
//    bool getData(float &xRate, float &yRate, float &zRate);
    bool getData(MPU9150_Data_Structure &IMU);
    void Push2Pack(Transmit_CommandPacket &_data);

private:
    void configureI2c();
    void configureDevice();
    void configureInterrupts();

    virtual void onInterruptExti0();
    virtual void onInterruptDma1Stream2();
    virtual void onInterruptI2c3Ev();

    char _rxBuffer[MPU9150_I2C_RX_BUFFER_SIZE];
    char _rxBuffer_tmp[MPU9150_I2C_RX_BUFFER_SIZE];

    bool _registerAddressSent;
    bool _dataUpdated;
    MPU9150_Data_Structure _data;

    EXTI_InitTypeDef   _extInitStructure;
};
