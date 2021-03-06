//-----------------------------------------------------------------------------
// MPU9150Driver.cpp
//
// This file contains the implementation of the I2C driver to communicate with
// the MPU9150 sensor
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------
#include "main.h"
#include "interruptTemplate.h"
//#include "MPU9150registers.h"
#include "mathRoutines.h" 
#include "MPU9150Driver.h"
#include <string.h>


//-----------------------------------------------------------------------------
// Sets the I2C port that is to be used for the functions of this driver and also
// sets up the appropriate interrupts and also the device itself
//-----------------------------------------------------------------------------
void MPU9150Driver::initialise()
{
    _dataUpdated  = false;

    //first configure the I2C
    configureI2c();

    //and then configure the device for operation
    configureDevice();

    //configure the interrupts
    configureInterrupts();
}

//-----------------------------------------------------------------------------
// Read the data if new stuff was received. Otherwise return false. All in a
// critical section
//-----------------------------------------------------------------------------
//bool MPU9150Driver::getData(float &xRate, float &yRate, float &zRate)
bool MPU9150Driver::getData(MPU9150_Data_Structure &IMU)
{
    bool bResult;
    disableInterrupts();
    if( _dataUpdated )
    {
        //xRate = (float)_data._gyroX * MPU9150_SENSITIVITY_LSB*DEG_TO_RAD;
        //yRate = (float)_data._gyroY * MPU9150_SENSITIVITY_LSB*DEG_TO_RAD;
        //zRate = (float)_data._gyroZ * MPU9150_SENSITIVITY_LSB*DEG_TO_RAD;
        IMU._gyroX = _data._gyroX * MPU9150_SENSITIVITY_LSB*DEG_TO_RAD;
        IMU._gyroY = _data._gyroY * MPU9150_SENSITIVITY_LSB*DEG_TO_RAD;
        IMU._gyroZ = _data._gyroZ * MPU9150_SENSITIVITY_LSB*DEG_TO_RAD;
        IMU._temperature = _data._temperature;
        IMU._accX = _data._accX;
        IMU._accY = _data._accY;
        IMU._accZ = _data._accZ;
        
        _dataUpdated = false;
        bResult = true;
    }else
        bResult = false;

    enableInterrupts();

    return bResult;
}
//-----------------------------------------------------------------------------
// Pull the data without checking interrupt. All in a
// critical section
//-----------------------------------------------------------------------------
void MPU9150Driver::Push2Pack(Transmit_CommandPacket &_data)
{
  char Buffer[21];
  char tmpchar=0;
  memset(Buffer,0,21);

  setRegister(MPU9150_ADDRESS,MPU9150_Int_Pin_CFG,2);
  setRegister(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
  // Check For DataReady
  getRegister(MPU9150_ADDRESS,0x3A,tmpchar);        
  if(tmpchar && 0x01)
    for(int ii = 59; ii<73 ; ii++)
      getRegister(MPU9150_ADDRESS,ii,Buffer[ii-59]);
  //tmpchar = 0;
  //getRegister(MPU9150_RA_MAG_ADDRESS,0x02,tmpchar);
  //if(tmpchar && 0x01)
    for(int ii = 3; ii<9 ; ii++)
      getRegister(MPU9150_RA_MAG_ADDRESS,ii,Buffer[ii+11]);
  _data.Acc_x = ((short int)Buffer[0]<<8)|Buffer[1];
  _data.Acc_y = ((short int)Buffer[2]<<8)|Buffer[3];
  _data.Acc_z = ((short int)Buffer[4]<<8)|Buffer[5];
  _data.Gyro_x = ((short int)Buffer[8]<<8)|Buffer[9];
  _data.Gyro_y = ((short int)Buffer[10]<<8)|Buffer[11];
  _data.Gyro_z = ((short int)Buffer[12]<<8)|Buffer[13];
  _data.Mag_x = ((short int)Buffer[15]<<8)|Buffer[14];
  _data.Mag_y = ((short int)Buffer[17]<<8)|Buffer[16];
  _data.Mag_z = ((short int)Buffer[19]<<8)|Buffer[18];
}
//-----------------------------------------------------------------------------
// Synchronously gets a register from the MPU9150. This function is expensive and
// should only be called for configuration
//-----------------------------------------------------------------------------
bool MPU9150Driver::getRegister(char DesAddress, char RegAddress, char &value)
{
    long int loops=10000;
    long int ii;
    //first generate a start command and wait for EV5 which is the end of the start bit
    //this checks the SB flag
    I2C_GenerateSTART(MPU9150_I2Cx, ENABLE);
    //while (!I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {}
    for(ii=0;ii<loops;++ii)
      if(I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
        break;
    if(ii == loops-1)  return(false);

    //now send the address to the device and wait for EV6 which is the end of the address transmission
    //and transmitter mode selected. this will be the ADDR flag being set. The CheckEvent function will
    //clear it by reading SR1 and SR2
    I2C_Send7bitAddress(MPU9150_I2Cx, DesAddress, I2C_Direction_Transmitter);
    //while (!I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {}
    for(ii=0;ii<loops;++ii)
      if(I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        break;
    if(ii == loops-1)  return(false);
    
    //send the register and wait for EV8_2 which means the byte has gone out
    I2C_SendData(MPU9150_I2Cx, RegAddress);
    //while (!I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {}
    for(ii=0;ii<loops;++ii)
      if(I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        break;
    if(ii == loops-1)  return(false);

    //generate a ReStart condition and wait for it to go through (the SB flag to be set)
    I2C_GenerateSTART(MPU9150_I2Cx, ENABLE);
    //while (!I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {}
    for(ii=0;ii<loops;++ii)
      if(I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
        break;
    if(ii == loops-1)  return(false);

    //select read mode after sending device address
    I2C_Send7bitAddress(MPU9150_I2Cx, DesAddress, I2C_Direction_Receiver);
    //set ACK to 0 so that NACK is set after the reception of the byte
    I2C_AcknowledgeConfig(MPU9150_I2Cx, DISABLE);

    //wait for one byte to be read in and then read it
//    while (!I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {}
    for(ii=0;ii<loops;++ii)
      if(I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
        break;
    if(ii == loops-1)  return(false);

    value = I2C_ReceiveData(MPU9150_I2Cx);

    //and now generate stop so that reception stops after first byte
    //not that this will not go through until the transfer is complete
    I2C_GenerateSTOP(MPU9150_I2Cx, ENABLE);

    //set ACK back to 1 so that we can continue per normal
    I2C_AcknowledgeConfig(MPU9150_I2Cx, ENABLE);

    return true;
}

//-----------------------------------------------------------------------------
// Synchronously sets a register on the MPU9150. This function is expensive and
// should only be called for configuration
//-----------------------------------------------------------------------------
void MPU9150Driver::setRegister(char DesAddress, char RegAddress, char value)
{
    //first generate a start command and wait for EV5 which is the end of the start bit
    //this means the SB flag will go high and then set back low by reading both SR1 and SR2
    I2C_GenerateSTART(MPU9150_I2Cx, ENABLE);
    while (!I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {}

    //now send the address to the device and wait for EV6 which is the end of the address transmission
    I2C_Send7bitAddress(MPU9150_I2Cx, DesAddress, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {}

    //send the register and wait for EV8
    //which means that the byte is in the shift register
    I2C_SendData(MPU9150_I2Cx, RegAddress );
    while (!I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {}

    //send the value and wait for EV8_2 which means everything has gone
    //out of the shift register and transmission is complete
    I2C_SendData(MPU9150_I2Cx, (char)value);
    while (!I2C_CheckEvent(MPU9150_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {}

    //and now generate stop condition
    I2C_GenerateSTOP(MPU9150_I2Cx, ENABLE);
}

//-----------------------------------------------------------------------------
// Sets the device registers and configures it for operation
//-----------------------------------------------------------------------------
void MPU9150Driver::configureDevice()
{
    //first read the power management register
    char val = 0;
    getRegister(MPU9150_ADDRESS,MPU9150_PWR_MGMT_1,val);

    //now disable sleep mode
    val &= ~0x40;
    setRegister(MPU9150_ADDRESS,MPU9150_PWR_MGMT_1,val);
    val = 0;
    // double check if sleep mode disabled
    //getRegister(MPU9150_ADDRESS,MPU9150_PWR_MGMT_1,val);
    
    // Enable interrupt when data is ready
    //val = 1;
    //setRegister(MPU9150_ADDRESS,MPU9150_Ext_IRQen,val);

/////////////////////////////////////////////////////
// Magnetometer stuff
/*
    // Enable Magnetometer Pass Through
    setRegister(MPU9150_ADDRESS,MPU9150_Int_Pin_CFG,0x02);

    //Enable Magnetometer
    setRegister(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
*////////////////////////////////////////////////////

//    char val = 0;
//    //reset the unit
//    setRegister(MPU9150_PWR_MGM,MPU9150_H_RESET);
//
//    Delay(1);
//
//    //set the IMU to use the X gyro as reference
//    getRegister(MPU9150_PWR_MGM,val);
//    setRegister(MPU9150_PWR_MGM,MPU9150_CLK_XGYRO);
//    getRegister(MPU9150_PWR_MGM,val);
//
//    //get the device address
//    getRegister(MPU9150_WHO_AM_I,val);
//
//    //wait for the PLL to be ready
//    while((val & MPU9150_IMU_RDY) == 0 )
//    {
//        val = 0;
//        //continuously poll for the register
//        getRegister(MPU9150_INT_STATUS,val);
//    }
//
//    //set the full scale sensitivity
//    setRegister(MPU9150_DLPF_FS,MPU9150_SENSITIVITY);
//
//    //set the interrupt pin to fire a 50uS pulse when new data is available
//    setRegister(MPU9150_INT_CFG,MPU9150_RAW_RDY_EN);
}

//-----------------------------------------------------------------------------
// Configure the GPIO interrupts and the I2C interrupts
//-----------------------------------------------------------------------------
void MPU9150Driver::configureInterrupts()
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    //and now register this class to receive the EXTI5 (specifically) interrupt
    //also register for I2C and DMA interrupts
    InterruptTemplate::registerForInterrupt(MPU9150_DRDY_PIN_EXT_PIN_HANDLER,this);
    InterruptTemplate::registerForInterrupt(I2C3EV_INTERRUPT_HANDLER,this);
    InterruptTemplate::registerForInterrupt(DMA1STREAM2_INTERRUPT_HANDLER,this);

    // Enable GPIOA clock
    RCC_AHB1PeriphClockCmd(MPU9150_DRDY_GPIO_CLK, ENABLE);
    // Enable SYSCFG clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // Configure PA5 pin as input floating so that we can capture the
    // INT pulse on PA5
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = MPU9150_DRDY_PIN;
    GPIO_Init(MPU9150_DRDY_GPIO_PORT, &GPIO_InitStructure);

    // Connect EXTI Line5 to PA5 pin
    SYSCFG_EXTILineConfig(MPU9150_DRDY_PIN_EXT_PORT_SOURCE, MPU9150_DRDY_PIN_EXT_PIN_SOURCE);

    // Configure EXTI Line5
    _extInitStructure.EXTI_Line = MPU9150_DRDY_PIN_EXT_PIN_LINE;
    _extInitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    _extInitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    _extInitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&_extInitStructure);

    /* Enable and set EXTI Line9-5 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = MPU9150_DRDY_PIN_EXT_PIN_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // enable the I2C2 event interrupt and priority
    NVIC_InitStructure.NVIC_IRQChannel = MPU9150_I2Cx_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // enable the DMA1 Channel7 interrupt and priority
    NVIC_InitStructure.NVIC_IRQChannel = MPU9150_I2Cx_DMA_STREAM_RX_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

//-----------------------------------------------------------------------------
// Configures the I2C and DMA for this driver
//-----------------------------------------------------------------------------
void MPU9150Driver::configureI2c()
{
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    DMA_InitTypeDef  DMA_InitStructure;

    // RCC Configuration
    //I2C Peripheral clock enable
    RCC_APB1PeriphClockCmd(MPU9150_I2Cx_CLK, ENABLE);

    //SDA GPIO clock enable
    RCC_AHB1PeriphClockCmd(MPU9150_I2Cx_SDA_GPIO_CLK, ENABLE);

    //SCL GPIO clock enable
    RCC_AHB1PeriphClockCmd(MPU9150_I2Cx_SCL_GPIO_CLK, ENABLE);

    // Reset I2Cx IP
    RCC_APB1PeriphResetCmd(MPU9150_I2Cx_CLK, ENABLE);
    RCC_APB1PeriphResetCmd(MPU9150_I2Cx_CLK, DISABLE);

    // Enable the DMA clock
    RCC_AHB1PeriphClockCmd(MPU9150_DMAx_CLK, ENABLE);

    //GPIO Configuration
    //Configure I2C SCL pin
    GPIO_InitStructure.GPIO_Pin = MPU9150_I2Cx_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(MPU9150_I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);

    //Configure I2C SDA pin
    GPIO_InitStructure.GPIO_Pin = MPU9150_I2Cx_SDA_PIN;
    GPIO_Init(MPU9150_I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);

    // Connect PXx to I2C_SCL
    GPIO_PinAFConfig(MPU9150_I2Cx_SCL_GPIO_PORT, MPU9150_I2Cx_SCL_SOURCE, MPU9150_I2Cx_AF);

    // Connect PXx to I2C_SDA
    GPIO_PinAFConfig(MPU9150_I2Cx_SDA_GPIO_PORT, MPU9150_I2Cx_SDA_SOURCE, MPU9150_I2Cx_AF);

    // Clear any pending flag on Rx Stream
    DMA_ClearFlag(MPU9150_I2Cx_DMA_STREAM_RX, MPU9150_I2Cx_RX_DMA_TCFLAG | MPU9150_I2Cx_RX_DMA_FEIFLAG | MPU9150_I2Cx_RX_DMA_DMEIFLAG | \
                  MPU9150_I2Cx_RX_DMA_TEIFLAG | MPU9150_I2Cx_RX_DMA_HTIFLAG);

    // Disable the I2C Rx DMA stream
    DMA_Cmd(MPU9150_I2Cx_DMA_STREAM_RX, DISABLE);
    // Configure the DMA stream for the I2C peripheral RX direction
    DMA_DeInit(MPU9150_I2Cx_DMA_STREAM_RX);

    // Initialize the DMA_Channel member
    DMA_InitStructure.DMA_Channel = MPU9150_I2Cx_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)MPU9150_I2Cx_DR_ADDRESS;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    // Init DMA for RX by setting the properties and calling the DeInit and then Init functions
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_rxBuffer;
    DMA_InitStructure.DMA_BufferSize = MPU9150_I2C_RX_BUFFER_SIZE;
    DMA_DeInit(MPU9150_I2Cx_DMA_STREAM_RX);
    //wait for the EN bit to go low. We cannot init before the EN bit
    //drops
    while(DMA_GetCmdStatus(MPU9150_I2Cx_DMA_STREAM_RX) == ENABLE)
    {}
    //now init the DMA.
    DMA_Init(MPU9150_I2Cx_DMA_STREAM_RX, &DMA_InitStructure);

    //disable both TX and RX
    DMA_Cmd(MPU9150_I2Cx_DMA_STREAM_RX, DISABLE);

    //enable the I2C
    I2C_Cmd(MPU9150_I2Cx, ENABLE);

    //set the I2c params and set it in master mode by issuing a START command
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_DutyCycle = MPU9150_I2C_DUTYCYCLE;
    I2C_InitStructure.I2C_ClockSpeed = MPU9150_I2C_SPEED;
    I2C_Init(MPU9150_I2Cx, &I2C_InitStructure);
}

//-----------------------------------------------------------------------------
// This function is called by the interrupt template when an interrupt is detected
// on Exti5
//-----------------------------------------------------------------------------
void MPU9150Driver::onInterruptExti0()
{
    //disable the exti interrupt
    _extInitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&_extInitStructure);

    //enable the I2C interrupt
    I2C_ITConfig(MPU9150_I2Cx, I2C_IT_EVT, ENABLE);

    //then we have new data which we need to read
    //generate the start condition. The rest will be handled by interrupts
    _registerAddressSent = false;
    I2C_GenerateSTART(MPU9150_I2Cx, ENABLE);
}

//-----------------------------------------------------------------------------
// This function is called by the interrupt template when an interrupt is detected
// on DMA1 stream 2
//-----------------------------------------------------------------------------
void MPU9150Driver::onInterruptDma1Stream2()
{
    //generate a stop condition on the I2C
    I2C_GenerateSTOP(MPU9150_I2Cx, ENABLE);

    //stop the DMA channel
    DMA_ITConfig(MPU9150_I2Cx_DMA_STREAM_RX,DMA_IT_TC,DISABLE);
    I2C_DMACmd(MPU9150_I2Cx, DISABLE);
    DMA_Cmd(MPU9150_I2Cx_DMA_STREAM_RX, DISABLE);

    //enable the exti interrupt so that we can receive data from the unit again
    _extInitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&_extInitStructure);

    //get the data from the DMA and copy it over to the local array
    
    for(int i=1;i<=MPU9150_I2C_RX_BUFFER_SIZE;i++)
      _rxBuffer_tmp[MPU9150_I2C_RX_BUFFER_SIZE-i] = _rxBuffer[i-1];
    MPU9150_Data_Structure *data = reinterpret_cast<MPU9150_Data_Structure*>((void *)_rxBuffer_tmp);
    _data = *data;
    _dataUpdated = true;
}

//-----------------------------------------------------------------------------
// This function is called by the interrupt template when an interrupt is detected
// on I2C3
//-----------------------------------------------------------------------------
void MPU9150Driver::onInterruptI2c3Ev()
{
    uint32_t event = I2C_GetLastEvent(MPU9150_I2Cx);
    switch (event)
    {
    case I2C_EVENT_MASTER_MODE_SELECT:
        //then we have sent the start bit. Depending on whether we have sent
        //the register address, either send the 7 bit address in read or write mode
        if( _registerAddressSent )
            I2C_Send7bitAddress(MPU9150_I2Cx, MPU9150_ADDRESS, I2C_Direction_Receiver);
        else
            I2C_Send7bitAddress(MPU9150_I2Cx, MPU9150_ADDRESS, I2C_Direction_Transmitter);
        break;

        //we have sent the initial 7 bit address and must now send the initial start
        //register address (which is accelx out high byte)
    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
        I2C_SendData(MPU9150_I2Cx, MPU9150_ACCEL_XOUT_H);
        break;

        //the register address has been sent. We need to generate
        //a new start
    case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
        _registerAddressSent = true;
        I2C_GenerateSTART(MPU9150_I2Cx, ENABLE);
        break;

        //we are at the point where we can start receiving via DMA
    case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
        // Clear any pending flag on Rx Stream
        DMA_ClearFlag(MPU9150_I2Cx_DMA_STREAM_RX, MPU9150_I2Cx_RX_DMA_TCFLAG | MPU9150_I2Cx_RX_DMA_FEIFLAG | MPU9150_I2Cx_RX_DMA_DMEIFLAG | \
                      MPU9150_I2Cx_RX_DMA_TEIFLAG | MPU9150_I2Cx_RX_DMA_HTIFLAG);

        //disable the I2C interrupt. The next interrupt
        //will be handled by the DMA
        I2C_ITConfig(MPU9150_I2Cx, I2C_IT_EVT, DISABLE);
        DMA_SetCurrDataCounter(MPU9150_I2Cx_DMA_STREAM_RX,MPU9150_DMA_READ_LENGTH);
        I2C_DMACmd(MPU9150_I2Cx, ENABLE);
        //make sure the next DMA transfer is the last so that NACK is sent at the end
        I2C_DMALastTransferCmd(MPU9150_I2Cx, ENABLE);

        DMA_Cmd(MPU9150_I2Cx_DMA_STREAM_RX, ENABLE);
        DMA_ITConfig(MPU9150_I2Cx_DMA_STREAM_RX,DMA_IT_TC,ENABLE);

        break;
    }
}