//-----------------------------------------------------------------------------
// LSM303Driver.cpp
//
// This file contains the implementation of the I2C driver to communicate with
// the LSM303DLH sensor
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------
#include "main.h"
#include "interruptTemplate.h"
#include "lsm303registers.h"
#include "lsm303driver.h"
namespace Andromeda
{
	//-----------------------------------------------------------------------------
	// Sets the I2C port that is to be used for the functions of this driver and also
	// sets up the appropriate interrupts and also the device itself
	//-----------------------------------------------------------------------------
	void LSM303Driver::initialise()
	{
		 _currentDmaOperation = LSM303_NONE;
        _nextDmaOperation = LSM303_NONE;

        _magDataUpdated = false;
        _accelDataUpdated = false;

		//first configure the I2C
        configureI2c();

		//and then configure the device for operation
		configureDevice();

		//configure the interrupts
		configureInterrupts();
	}

    //-----------------------------------------------------------------------------
	// Read the data if new stuff was received. Otherwise return false. All in a 
	// critical section. Returns in 
	//-----------------------------------------------------------------------------
    bool LSM303Driver::getAccelData(float &xAccel, float &yAccel, float &zAccel)
	{
		bool bResult;
		disableInterrupts();
		if( _accelDataUpdated )
		{
			xAccel = (float)_accelData._accelX * LSM_ACCEL_SENSITIVITY_LSB;
            yAccel = (float)_accelData._accelY * LSM_ACCEL_SENSITIVITY_LSB;
            zAccel = (float)_accelData._accelZ * LSM_ACCEL_SENSITIVITY_LSB;

            _accelDataUpdated = false;
			bResult = true;
		}else
			bResult = false;
		
        enableInterrupts();

		return bResult;
	}

    //-----------------------------------------------------------------------------
	// Read the data if new stuff was received. Otherwise return false. All in a 
	// critical section 
	//-----------------------------------------------------------------------------
    bool LSM303Driver::getMagData(float &xMag, float &yMag, float &zMag)
	{
		bool bResult;
		disableInterrupts();
		if( _magDataUpdated )
		{
			xMag = (float)_magData._magX * LSM_MAG_SENSITIVITY_LSB_XY;
            yMag = (float)_magData._magY * LSM_MAG_SENSITIVITY_LSB_XY;
            zMag = (float)_magData._magZ * LSM_MAG_SENSITIVITY_LSB_Z;

            _magDataUpdated  = false;
			bResult = true;
		}else
			bResult = false;
		
        enableInterrupts();

		return bResult;
	}
	
	//-----------------------------------------------------------------------------
	// Synchronously gets a register from the LSM303. This function is expensive and
	// should only be called for configuration
	//-----------------------------------------------------------------------------
    bool LSM303Driver::getRegister(char deviceAddress, char address, char &value)
	{
		//first generate a start command and wait for EV5 which is the end of the start bit
		//this checks the SB flag
		I2C_GenerateSTART(LSM303_I2Cx, ENABLE);
		while (!I2C_CheckEvent(LSM303_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {}

		//now send the address to the device and wait for EV6 which is the end of the address transmission
		//and transmitter mode selected. this will be the ADDR flag being set. The CheckEvent function will
		//clear it by reading SR1 and SR2
		I2C_Send7bitAddress(LSM303_I2Cx, deviceAddress, I2C_Direction_Transmitter);
        while (!I2C_CheckEvent(LSM303_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {}

		//send the register and wait for EV8_2 which means the byte has gone out
        I2C_SendData(LSM303_I2Cx, address);
		while (!I2C_CheckEvent(LSM303_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {}

		//generate a ReStart condition and wait for it to go through (the SB flag to be set)
        I2C_GenerateSTART(LSM303_I2Cx, ENABLE);
		while (!I2C_CheckEvent(LSM303_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {}

		//select read mode after sending device address
        I2C_Send7bitAddress(LSM303_I2Cx, deviceAddress, I2C_Direction_Receiver);
        //set ACK to 0 so that NACK is set after the reception of the byte
        I2C_AcknowledgeConfig(LSM303_I2Cx, DISABLE);
		
		//wait for one byte to be read in and then read it
        while (!I2C_CheckEvent(LSM303_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {}
        value = I2C_ReceiveData(LSM303_I2Cx);

        //and now generate stop so that reception stops after first byte
		//not that this will not go through until the transfer is complete
		I2C_GenerateSTOP(LSM303_I2Cx, ENABLE);

		//set ACK back to 1 so that we can continue per normal
        I2C_AcknowledgeConfig(LSM303_I2Cx, ENABLE);
		
		return true;
	}

    //-----------------------------------------------------------------------------
	// Synchronously sets a register on the LSM303. This function is expensive and
	// should only be called for configuration
	//-----------------------------------------------------------------------------
	void LSM303Driver::setRegister(char deviceAddress, char address, char value)
	{
		//first generate a start command and wait for EV5 which is the end of the start bit
		//this means the SB flag will go high and then set back low by reading both SR1 and SR2
		I2C_GenerateSTART(LSM303_I2Cx, ENABLE);
		while (!I2C_CheckEvent(LSM303_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {}

		//now send the address to the device and wait for EV6 which is the end of the address transmission
		I2C_Send7bitAddress(LSM303_I2Cx, deviceAddress, I2C_Direction_Transmitter);
        while (!I2C_CheckEvent(LSM303_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {}

		//send the register and wait for EV8
		//which means that the byte is in the shift register
        I2C_SendData(LSM303_I2Cx, address );
		while (!I2C_CheckEvent(LSM303_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {}

        //send the value and wait for EV8_2 which means everything has gone
		//out of the shift register and transmission is complete
        I2C_SendData(LSM303_I2Cx, (char)value);
		while (!I2C_CheckEvent(LSM303_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {}

        //and now generate stop condition
		I2C_GenerateSTOP(LSM303_I2Cx, ENABLE);
	}

	//-----------------------------------------------------------------------------
	// Sets the device registers and configures it for operation
	//-----------------------------------------------------------------------------
	void LSM303Driver::configureDevice()
	{
		
		//configure the magnetometer
		char val = 0;

		//set the data output rate to 75Hz
		setRegister(LSM303_ADDRESS_M,LSM303_CRA_REG_M, LSM303_M_7_5HZ|LSM303_M_NORMAL_MEASUREMENT);
        getRegister(LSM303_ADDRESS_M,LSM303_CRA_REG_M,val);

		//take the device out of sleep mode
		setRegister(LSM303_ADDRESS_M,LSM303_MR_REG_M, LSM303_M_CONTINUOUS_CONVERSION);
        getRegister(LSM303_ADDRESS_M,LSM303_MR_REG_M,val);

		//set the gain to 1.3 gauss overall
        setRegister(LSM303_ADDRESS_M,LSM303_CRB_REG_M, LSM_ACCEL_SENSITIVITY);

		//read out the status
        getRegister(LSM303_ADDRESS_M,LSM303_SR_REG_Mg,val);
			
		//configure the accelerometer

		//first restart the accelerometer
		setRegister(LSM303_ADDRESS_A,LSM303_CTRL_REG2_A, LSM303_A_BOOT);

        Delay(1);

		//set capture rate to 10Hz lower power, set frequency cutoff to 37hz 
		//and enable all channels
		setRegister(LSM303_ADDRESS_A,LSM303_CTRL_REG1_A, LSM303_A_LOW_POWER_10HZ|LSM303_A_ODR50_LP37|LSM303_A_ZEN|LSM303_A_YEN|LSM303_A_XEN);
		//configure interrupts so that INT2 fires on DRDY
        setRegister(LSM303_ADDRESS_A,LSM303_CTRL_REG3_A, LSM303_A_I2CFG_BOOT_RUNNING|LSM303_A_I1CFG_DATA_READY);
		//set full scale to 2g and disable data updates when we read from the accelerometer
        setRegister(LSM303_ADDRESS_A,LSM303_CTRL_REG4_A, LSM_MAG_SENSITIVITY|LSM303_A_BDU);
		//read out the status
        getRegister(LSM303_ADDRESS_A,LSM303_STATUS_REG_A,val);
		//clear interrupts
        getRegister(LSM303_ADDRESS_A,LSM303_INT2_SOURCE_A,val);
		
	}

	//-----------------------------------------------------------------------------
	// Configure the GPIO interrupts and the I2C interrupts
	//-----------------------------------------------------------------------------
    void LSM303Driver::configureInterrupts()
	{
		EXTI_InitTypeDef   EXTI_InitStructure;
		GPIO_InitTypeDef   GPIO_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;
		
        //and now register this class to receive the EXTI0 and EXTI4 (specifically) interrupt
        InterruptTemplate::registerForInterrupt(LSM303_DRDY_A_PIN_EXT_PIN_HANDLER,this);
        InterruptTemplate::registerForInterrupt(LSM303_DRDY_M_PIN_EXT_PIN_HANDLER,this);
        InterruptTemplate::registerForInterrupt(I2C1EV_INTERRUPT_HANDLER,this);
        InterruptTemplate::registerForInterrupt(DMA1STREAM0_INTERRUPT_HANDLER,this);

		// Enable GPIOA clock 
		RCC_AHB1PeriphClockCmd(LSM303_DRDY_A_GPIO_CLK, ENABLE);
        RCC_AHB1PeriphClockCmd(LSM303_DRDY_M_GPIO_CLK, ENABLE);
		// Enable SYSCFG clock 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		
		// Configure PA0 and PA4 pins as input floating so that we can capture the
		// INT pulse on PA0 and PA4
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Pin = LSM303_DRDY_A_PIN;
		GPIO_Init(LSM303_DRDY_A_GPIO_PORT, &GPIO_InitStructure);
		
        GPIO_InitStructure.GPIO_Pin = LSM303_DRDY_M_PIN;
        GPIO_Init(LSM303_DRDY_M_GPIO_PORT, &GPIO_InitStructure);
		
		// Connect EXTI Line1 to PA1 pin 
        // Connect EXTI Line4 to PA4 pin 
		SYSCFG_EXTILineConfig(LSM303_DRDY_A_PIN_EXT_PORT_SOURCE, LSM303_DRDY_A_PIN_EXT_PIN_SOURCE);
        SYSCFG_EXTILineConfig(LSM303_DRDY_M_PIN_EXT_PORT_SOURCE, LSM303_DRDY_M_PIN_EXT_PIN_SOURCE);
		
		// Configure EXTI Line1 for the accelerometer DRDY
        // Configure EXTI Line4 for the magnetometer DRDY
		_extInitStructure.EXTI_Line = LSM303_DRDY_A_PIN_EXT_PIN_LINE;
		_extInitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		_extInitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
		_extInitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&_extInitStructure);

		_extInitStructure.EXTI_Line = LSM303_DRDY_M_PIN_EXT_PIN_LINE;
        EXTI_Init(&_extInitStructure);
		
		/* Enable and set EXTI Line0 Interrupt to the lowest priority */
        /* Enable and set EXTI Line4 Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = LSM303_DRDY_A_PIN_EXT_PIN_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

        NVIC_InitStructure.NVIC_IRQChannel = LSM303_DRDY_M_PIN_EXT_PIN_IRQ;
        NVIC_Init(&NVIC_InitStructure);

        // enable the I2C1 event interrupt and priority
		NVIC_InitStructure.NVIC_IRQChannel = LSM303_I2Cx_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

        // enable the DMA1 Channel7 interrupt and priority
		NVIC_InitStructure.NVIC_IRQChannel = LSM303_I2Cx_DMA_STREAM_RX_IRQ;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		
	}

	//-----------------------------------------------------------------------------
	// Configures the I2C and DMA for this driver
	//-----------------------------------------------------------------------------
	void LSM303Driver::configureI2c()
	{
		I2C_InitTypeDef  I2C_InitStructure;
		GPIO_InitTypeDef  GPIO_InitStructure;
        DMA_InitTypeDef  DMA_InitStructure;
	
		// RCC Configuration 
		//I2C Peripheral clock enable 
		RCC_APB1PeriphClockCmd(LSM303_I2Cx_CLK, ENABLE);
		
		//SDA GPIO clock enable 
		RCC_AHB1PeriphClockCmd(LSM303_I2Cx_SDA_GPIO_CLK, ENABLE);
		
		//SCL GPIO clock enable 
		RCC_AHB1PeriphClockCmd(LSM303_I2Cx_SCL_GPIO_CLK, ENABLE);
		
		// Reset I2Cx IP 
		RCC_APB1PeriphResetCmd(LSM303_I2Cx_CLK, ENABLE);
		RCC_APB1PeriphResetCmd(LSM303_I2Cx_CLK, DISABLE);
		
		// Enable the DMA clock 
		RCC_AHB1PeriphClockCmd(LSM303_DMAx_CLK, ENABLE);
		
		// GPIO Configuration 
		//Configure I2C SCL pin 
		GPIO_InitStructure.GPIO_Pin = LSM303_I2Cx_SCL_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(LSM303_I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);
		
		// Configure I2C SDA pin 
		GPIO_InitStructure.GPIO_Pin = LSM303_I2Cx_SDA_PIN;
		GPIO_Init(LSM303_I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);
		
		// Connect PXx to I2C_SCL 
		GPIO_PinAFConfig(LSM303_I2Cx_SCL_GPIO_PORT, LSM303_I2Cx_SCL_SOURCE, LSM303_I2Cx_SCL_AF);
		
		// Connect PXx to I2C_SDA 
		GPIO_PinAFConfig(LSM303_I2Cx_SDA_GPIO_PORT, LSM303_I2Cx_SDA_SOURCE, LSM303_I2Cx_SDA_AF);
		
		// DMA Configuration 
		// Clear any pending flag on Rx Stream  
		DMA_ClearFlag(LSM303_I2Cx_DMA_STREAM_RX, LSM303_I2Cx_RX_DMA_TCFLAG | LSM303_I2Cx_RX_DMA_FEIFLAG | LSM303_I2Cx_RX_DMA_DMEIFLAG | \
										   LSM303_I2Cx_RX_DMA_TEIFLAG | LSM303_I2Cx_RX_DMA_HTIFLAG);
		
		// Disable the I2C Rx DMA stream 
		DMA_Cmd(LSM303_I2Cx_DMA_STREAM_RX, DISABLE);
		// Configure the DMA stream for the I2C peripheral RX direction 
		DMA_DeInit(LSM303_I2Cx_DMA_STREAM_RX);
		
		// Initialize the DMA_Channel member 
		DMA_InitStructure.DMA_Channel = LSM303_I2Cx_DMA_CHANNEL;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)LSM303_I2Cx_DR_ADDRESS;
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
		DMA_InitStructure.DMA_BufferSize = LSM303_I2C_RX_BUFFER_SIZE;
		DMA_DeInit(LSM303_I2Cx_DMA_STREAM_RX);
		//wait for the EN bit to go low
		while(DMA_GetCmdStatus(LSM303_I2Cx_DMA_STREAM_RX) == ENABLE)
		{}

		DMA_Init(LSM303_I2Cx_DMA_STREAM_RX, &DMA_InitStructure);
		
		//disable both TX and RX
        DMA_Cmd(LSM303_I2Cx_DMA_STREAM_RX, DISABLE);
		
		//enable the I2C
		I2C_Cmd(LSM303_I2Cx, ENABLE);

		//set the I2c params and set it in master mode by issuing a START command
        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure.I2C_DutyCycle = LSM303_I2C_DUTYCYCLE;
		I2C_InitStructure.I2C_ClockSpeed = LSM303_I2C_SPEED;
		I2C_Init(LSM303_I2Cx, &I2C_InitStructure);	
	}
	
	//-----------------------------------------------------------------------------
	// Begins DMA operations to read the accelerometer
	//-----------------------------------------------------------------------------
    void LSM303Driver::startAccelerometerDma()
	{
		//if the DMA is active, just set as the next operation
		if(_currentDmaOperation != LSM303_NONE )
		{
			_nextDmaOperation = LSM303_ACCELEROMETER;
			return;
		}
        _currentDmaOperation = LSM303_ACCELEROMETER;

		//disable the exti1 interrupt 
        _extInitStructure.EXTI_Line = LSM303_DRDY_A_PIN_EXT_PIN_LINE;
		_extInitStructure.EXTI_LineCmd = DISABLE;
		EXTI_Init(&_extInitStructure);

        //enable the I2C interrupt
		I2C_ITConfig(LSM303_I2Cx, I2C_IT_EVT, ENABLE);

		//then we have new data which we need to read
		//generate the start condition. The rest will be handled by interrupts
        _registerAddressSent = false;
		I2C_GenerateSTART(LSM303_I2Cx, ENABLE);
	}

	//-----------------------------------------------------------------------------
	// Begins DMA operations to read the magnetometer
	//-----------------------------------------------------------------------------
    void LSM303Driver::startMagnetometerDma()
	{
		//if the DMA is active, just set as the next operation
		if(_currentDmaOperation != LSM303_NONE )
		{
			_nextDmaOperation = LSM303_MAGNETOMETER;
			return;
		}
        _currentDmaOperation = LSM303_MAGNETOMETER;
		//set next one to accelerometer to read both
        _nextDmaOperation = LSM303_ACCELEROMETER;
			
		//disable the exti1 interrupt 
        _extInitStructure.EXTI_Line = LSM303_DRDY_M_PIN_EXT_PIN_LINE;
		_extInitStructure.EXTI_LineCmd = DISABLE;
		EXTI_Init(&_extInitStructure);

        //enable the I2C interrupt
		I2C_ITConfig(LSM303_I2Cx, I2C_IT_EVT, ENABLE);

		//then we have new data which we need to read
		//generate the start condition. The rest will be handled by interrupts
        _registerAddressSent = false;
		I2C_GenerateSTART(LSM303_I2Cx, ENABLE);
	}

	
    //-----------------------------------------------------------------------------
	// This function is called by the interrupt template when an interrupt is detected
	// on Exti1 which is the DRDY pin for the accelerometer
	//-----------------------------------------------------------------------------
    void LSM303Driver::onInterruptExti1()
	{	
		startAccelerometerDma();
	}

    //-----------------------------------------------------------------------------
	// This function is called by the interrupt template when an interrupt is detected
	// on Exti4 which is the DRDY pin for the accelerometer
	//-----------------------------------------------------------------------------
    void LSM303Driver::onInterruptExti4()
	{	
		startMagnetometerDma();
	}

    //-----------------------------------------------------------------------------
	// This function is called by the interrupt template when an interrupt is detected
	// on DMA1 channel 7
	//-----------------------------------------------------------------------------
    void LSM303Driver::onInterruptDma1Stream0()
	{	
		//generate a stop condition on the I2C
        I2C_GenerateSTOP(LSM303_I2Cx, ENABLE);

		//stop the DMA channel
        DMA_ITConfig(LSM303_I2Cx_DMA_STREAM_RX,DMA_IT_TC,DISABLE);
        I2C_DMACmd(LSM303_I2Cx, DISABLE);
		DMA_Cmd(LSM303_I2Cx_DMA_STREAM_RX, DISABLE);

		//depending on what was read, set the appropriate flags
		if( _currentDmaOperation == LSM303_ACCELEROMETER)
		{
			//get the data from the DMA and copy it over to the local array
			LSM303_Accel_Data_Structure *data = reinterpret_cast<LSM303_Accel_Data_Structure*>((void *)_rxBuffer);
			//right shift the data by 4 ( as it's 12 bit )
//			data->_accelX = data->_accelX >> 4;
//            data->_accelY = data->_accelY >> 4;
//            data->_accelZ = data->_accelZ >> 4;

			_accelData = *data;
			_accelDataUpdated = true;
        }else if(_currentDmaOperation == LSM303_MAGNETOMETER)
		{
			//get the data from the DMA and copy it over to the local array
			LSM303_Mag_Data_Structure *data = reinterpret_cast<LSM303_Mag_Data_Structure*>((void *)_rxBuffer);
			_magData = *data;
			_magDataUpdated = true;
		}

		//once we have finished the operation, if there is another read
		//pending, start it now

		_currentDmaOperation = LSM303_NONE;
		if( _nextDmaOperation == LSM303_ACCELEROMETER )
		{
			_nextDmaOperation = LSM303_NONE;
			startAccelerometerDma();
		} else if( _nextDmaOperation == LSM303_ACCELEROMETER )
		{
			_nextDmaOperation = LSM303_NONE;
			startMagnetometerDma();
		}else
		{
			//enable the exti interrupt so that we can receive data from the unit again 
			if( _currentDmaOperation == LSM303_ACCELEROMETER )
				_extInitStructure.EXTI_Line = LSM303_DRDY_A_PIN_EXT_PIN_LINE;
			else
				_extInitStructure.EXTI_Line = LSM303_DRDY_M_PIN_EXT_PIN_LINE;
			
			_extInitStructure.EXTI_LineCmd = ENABLE;
			EXTI_Init(&_extInitStructure);
		}

		
			

		//get the data from the DMA
//        IMU3000_Data_Structure *data = reinterpret_cast<IMU3000_Data_Structure*>((void *)_rxBuffer);
//		int i = data->_temperature;
	}

    //-----------------------------------------------------------------------------
	// This function is called by the interrupt template when an interrupt is detected
	// on I2C1
	//-----------------------------------------------------------------------------
	void LSM303Driver::onInterruptI2c1Ev()
	{
		char address;
		char reg;
		if( _currentDmaOperation == LSM303_ACCELEROMETER )
		{
			address = LSM303_ADDRESS_A;
			reg = LSM303_OUT_X_L_A | 0x80 ; //set the most significant bit to 1 to read multiple bytes
		}
		else
		{
			address = LSM303_ADDRESS_M;
			reg = LSM303_OUT_X_H_M | 0x80;
		}


		uint32_t event = I2C_GetLastEvent(LSM303_I2Cx);
		switch (event)
		{
			case I2C_EVENT_MASTER_MODE_SELECT:
				//then we have sent the start bit. Depending on whether we have sent
				//the register address, either send the 7 bit address in read or write mode
				if( _registerAddressSent )
					I2C_Send7bitAddress(LSM303_I2Cx, address, I2C_Direction_Receiver);
				else
					I2C_Send7bitAddress(LSM303_I2Cx, address, I2C_Direction_Transmitter);
				break;

			//we have sent the initial 7 bit address and must now send the initial start
			//register address
			case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
                I2C_SendData(LSM303_I2Cx, reg);
				break;

			//the register address has been sent. We need to generate
			//a new start 
			case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
				_registerAddressSent = true;
                I2C_GenerateSTART(LSM303_I2Cx, ENABLE);
				break;

			//we are at the point where we can start receiving via DMA
			case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
				// Clear any pending flag on Rx Stream  
				DMA_ClearFlag(LSM303_I2Cx_DMA_STREAM_RX, LSM303_I2Cx_RX_DMA_TCFLAG | LSM303_I2Cx_RX_DMA_FEIFLAG | LSM303_I2Cx_RX_DMA_DMEIFLAG | \
										   LSM303_I2Cx_RX_DMA_TEIFLAG | LSM303_I2Cx_RX_DMA_HTIFLAG);

				//disable the I2C interrupt. The next interrupt
				//will be handled by the DMA
				I2C_ITConfig(LSM303_I2Cx, I2C_IT_EVT, DISABLE); 

				DMA_SetCurrDataCounter(LSM303_I2Cx_DMA_STREAM_RX,LSM303_A_DMA_READ_LENGTH);
				I2C_DMACmd(LSM303_I2Cx, ENABLE);
				//make sure the next DMA transfer is the last so that NACK is sent at the end
                I2C_DMALastTransferCmd(LSM303_I2Cx, ENABLE);

				DMA_Cmd(LSM303_I2Cx_DMA_STREAM_RX, ENABLE);
                DMA_ITConfig(LSM303_I2Cx_DMA_STREAM_RX,DMA_IT_TC,ENABLE);
				
				break;
		}
	}
}