//-----------------------------------------------------------------------------
// AhrsSpiDriver.cpp
//
// This file contains the implementation of the UART driver to communicate with
// the AHRS module of the andromeda autopilot
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------

#include "main.h"
#include "InterruptTemplate.h"
#include "AhrsSpiDriver.h"
#include "AhrsModuleDefinitions.h"

using namespace Andromeda;

//-----------------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------------
AhrsSpiDriver::AhrsSpiDriver()
{
	_rxTcFlag = AHRS_RX_DMA_FLAG_TCIF;
    _txTcFlag = AHRS_TX_DMA_FLAG_TCIF;

	//configure the transmission counters
    _rxFull = 0;
    _txFull = 0;
    _lastTx = -1;

    _txStream = AHRS_TX_DMA_STREAM;
    _rxStream = AHRS_RX_DMA_STREAM;
    
    DMA_ClearFlag(_txStream, _rxTcFlag);
	DMA_ClearFlag(_rxStream, _rxTcFlag);

	_currentCommand = AHRS_COMMAND_NONE;
}

//-----------------------------------------------------------------------------
// Configures DMA for the SPI port
//-----------------------------------------------------------------------------
void AhrsSpiDriver::configureDma()
{
    DMA_InitTypeDef DMA_InitStructure;

	DMA_StructInit(&DMA_InitStructure);

	//setup the DMA for transmission and reception
	//configure the DMA clock
    RCC_AHB1PeriphClockCmd(AHRS_DMAx_CLK, ENABLE);  
    
    //setup the DMA initialization structure
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)AHRS_DR_ADDRESS;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    //initialise the RX DMA with the RX0 buffer
    DMA_DeInit(AHRS_RX_DMA_STREAM);
    //wait for the EN bit to go low
    while(DMA_GetCmdStatus(AHRS_RX_DMA_STREAM) == ENABLE)
    {}
    
    DMA_InitStructure.DMA_Channel = AHRS_RX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_rxBuffer;
    DMA_InitStructure.DMA_BufferSize = (uint16_t)SPI_BUFFER_SIZE;
    DMA_Init(AHRS_RX_DMA_STREAM, &DMA_InitStructure);
    
    //initialise the TX DMA with the TX buffer
    DMA_DeInit(AHRS_TX_DMA_STREAM);
    //wait for the EN bit to go low
    while(DMA_GetCmdStatus(AHRS_TX_DMA_STREAM) == ENABLE)
    {}
    DMA_InitStructure.DMA_Channel = AHRS_TX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_txBuffer;
    DMA_InitStructure.DMA_BufferSize = 0; //no TX data to begin with
    DMA_Init(AHRS_TX_DMA_STREAM, &DMA_InitStructure);
    
    //disable both DMAs
    DMA_Cmd(AHRS_TX_DMA_STREAM, DISABLE);
    DMA_Cmd(AHRS_RX_DMA_STREAM, DISABLE);
    
    //Disable double buffer mode
    DMA_DoubleBufferModeCmd(AHRS_RX_DMA_STREAM, DISABLE);
     
    //enable the DMA IRQ requests for the UART for both
    //rx and tx
    SPI_I2S_DMACmd(AHRS_SPI, SPI_DMAReq_Rx, ENABLE);
    SPI_I2S_DMACmd(AHRS_SPI, SPI_DMAReq_Tx, ENABLE);
    
    DMA_Cmd(AHRS_RX_DMA_STREAM, ENABLE);
    
    //enable TC interrupt for the RX stream
    DMA_ITConfig(AHRS_RX_DMA_STREAM,DMA_IT_TC,ENABLE);

    DMA_ClearFlag(AHRS_RX_DMA_STREAM, AHRS_RX_DMA_FLAG_TCIF);
    DMA_ClearFlag(AHRS_TX_DMA_STREAM, AHRS_TX_DMA_FLAG_TCIF);
}

//-----------------------------------------------------------------------------
// Configures the SPI port
//-----------------------------------------------------------------------------
void AhrsSpiDriver::configureSpi()
{
	//initialise the SPI master connection
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
	
    GPIO_StructInit(&GPIO_InitStructure);
    SPI_StructInit(&SPI_InitStructure);

	// Enable the SPI clock 
	AHRS_SPI_CLK_INIT(AHRS_SPI_CLK, ENABLE);
	
	// Enable GPIO clocks 
	RCC_AHB1PeriphClockCmd(AHRS_SPI_SCK_GPIO_CLK | AHRS_SPI_MISO_GPIO_CLK | AHRS_SPI_MOSI_GPIO_CLK | AHRS_SPI_NSS_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	
	// SPI SCK pin configuration 
	GPIO_InitStructure.GPIO_Pin = AHRS_SPI_SCK_PIN;
	GPIO_Init(AHRS_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
	
	// SPI  MOSI pin configuration 
	GPIO_InitStructure.GPIO_Pin =  AHRS_SPI_MOSI_PIN;
	GPIO_Init(AHRS_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

	// SPI NSS pin configuration 
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin =  AHRS_SPI_NSS_PIN;
	GPIO_Init(AHRS_SPI_NSS_GPIO_PORT, &GPIO_InitStructure);

	// SPI  MISO pin configuration 
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin =  AHRS_SPI_MISO_PIN;
	GPIO_Init(AHRS_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

    // Connect SPI pins to AF5   
	GPIO_PinAFConfig(AHRS_SPI_SCK_GPIO_PORT, AHRS_SPI_SCK_SOURCE, AHRS_SPI_SCK_AF);
	GPIO_PinAFConfig(AHRS_SPI_MISO_GPIO_PORT, AHRS_SPI_MISO_SOURCE, AHRS_SPI_MISO_AF);
    GPIO_PinAFConfig(AHRS_SPI_MOSI_GPIO_PORT, AHRS_SPI_MOSI_SOURCE, AHRS_SPI_MOSI_AF);
    GPIO_PinAFConfig(AHRS_SPI_NSS_GPIO_PORT, AHRS_SPI_NSS_SOURCE, AHRS_SPI_NSS_AF);

	
	// SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(AHRS_SPI);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	//configure DMA for the SPI port
	configureDma();
	
	// Initializes the SPI communication
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_Init(AHRS_SPI, &SPI_InitStructure);
	
	// The Data transfer is performed in the SPI interrupt routine 
	// Enable the SPI peripheral 
	SPI_Cmd(AHRS_SPI, ENABLE);

     // Enable the Rx buffer not empty interrupt, which means that an interrupt
	 //will be thrown when a byte is received
	SPI_I2S_ITConfig(AHRS_SPI, SPI_I2S_IT_RXNE, ENABLE);
}

//-----------------------------------------------------------------------------
// Sets up interrupts to do with the SPI port and DMA
//-----------------------------------------------------------------------------
void AhrsSpiDriver::configureInterrupts()
{
	//and now register this class to receive the RX and TX interrupts
    InterruptTemplate::registerForInterrupt(DMA2STREAM3_INTERRUPT_HANDLER,this);
    InterruptTemplate::registerForInterrupt(DMA2STREAM4_INTERRUPT_HANDLER,this);
    
    NVIC_InitTypeDef   NVIC_InitStructure;
    // enable the RX Dma interrupts
    NVIC_InitStructure.NVIC_IRQChannel = AHRS_SPI_DMA_RX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    //  enable the TX Dma interrupts
    NVIC_InitStructure.NVIC_IRQChannel = AHRS_SPI_DMA_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// configure the SPI interrupt
	NVIC_InitStructure.NVIC_IRQChannel = AHRS_SPI_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//-----------------------------------------------------------------------------
// SPI2 interrupt when a byte is received.
//-----------------------------------------------------------------------------
void AhrsSpiDriver::onInterruptSpi2()
{
	//see if a new byte has been received
	if (SPI_I2S_GetITStatus(AHRS_SPI, SPI_I2S_IT_RXNE) == SET && _currentCommand == AHRS_COMMAND_NONE)
	{
		char byte = SPI_I2S_ReceiveData(AHRS_SPI);
		bool valid = false;

		//since an interrupt has been generated, this must be a command
		switch(byte)
		{
			case AHRS_COMMAND_POLL:
				//then a poll has been requested. We must output the values of the current poll via DMA
				memcpy(_txBuffer, _data,sizeof(_data));
				DMA_SetCurrDataCounter(_txStream,sizeof(_data));
				DMA_Cmd(_txStream, ENABLE);

				//since this command has no additional data, we can go straight to the TX dma and 
				//queue up the reply.
				_currentCommand = byte;
				//enable the DMA interrupt
				DMA_ITConfig(_txStream,DMA_IT_TC,ENABLE);
				//disable single byte interrupts
				SPI_I2S_ITConfig(AHRS_SPI, SPI_I2S_IT_RXNE, DISABLE);

				break;
		}
	}
}

//-----------------------------------------------------------------------------
// RX DMA interrupt
//-----------------------------------------------------------------------------
void AhrsSpiDriver::onInterruptDma1Stream3()
{

}

//-----------------------------------------------------------------------------
// TX DMA interrupt
//-----------------------------------------------------------------------------
void AhrsSpiDriver::onInterruptDma1Stream4()
{
	//if the TX is finished, it means that the current command has ended
	_currentCommand = AHRS_COMMAND_NONE;

	//disable further DMA interrupts
    DMA_ITConfig(_txStream,DMA_IT_TC,DISABLE);

	//enable next single byte interrupt
    SPI_I2S_ITConfig(AHRS_SPI, SPI_I2S_IT_RXNE, ENABLE);
}

//-----------------------------------------------------------------------------
// Default initialisation routine
//-----------------------------------------------------------------------------
void AhrsSpiDriver::initialise()
{
	//set up the SPI port
	configureSpi();
}

//-----------------------------------------------------------------------------
// Writes the given data to the TX buffer and initiates the send operation if 
// possible. Otherwise the data will be sent as soon as the previous send operation
// is finished
//-----------------------------------------------------------------------------
bool AhrsSpiDriver::beginWritePacket(char *data, short length)
{
    //this is a critical section so we start by disabling interrupts
    /***************************************************/
    /*             Start Critical Section              */
    /***************************************************/
    disableInterrupts();
    //make sure there is enough space
    if( (length + _txFull) > SPI_BUFFER_SIZE )
    {
	    _txFull = 0;  //if the buffer overflows, we must empty it so we can keep sending
	    enableInterrupts();
	    //enable the ENDTX interrupt so this stuff actually gets sent
	    DMA_ITConfig(_txStream,DMA_IT_TC,ENABLE);

	    return 0;
    }

    //and now copy data tot the tx buffer
    memmove(&_txBuffer[_txFull], data, length);
    _txFull += length;

    /***************************************************/
    /*             End Critical Section                */
    /***************************************************/
    //and reenable the interrupts
    enableInterrupts();

    //check to see if we have already finished, if so jump straight to the function,
    //otherwise enable the end interrupt
    if(isTransmitting())
	    DMA_ITConfig(_txStream,DMA_IT_TC,ENABLE);
    else
	    onInterruptDma2Stream3();

    return true;
}

//-----------------------------------------------------------------------------
// Returns a boolean value which indicates whether the SPI is transmitting or not
//-----------------------------------------------------------------------------
bool AhrsSpiDriver::isTransmitting()
{
	return (DMA_GetCmdStatus(_txStream) == ENABLE && DMA_GetCurrDataCounter(_txStream)!=0);
}

//-----------------------------------------------------------------------------
// Polls the AHRS which returns all heading and attitude information via SPI.
// The results are asynchronously placed in the holding structures of this class
//-----------------------------------------------------------------------------
void AhrsSpiDriver::poll()
{
	//send the poll command to the module
	char transmitCommand[AHRS_COMMAND_POLL_LENGTH];
	transmitCommand[0] = AHRS_COMMAND_POLL;
	
	beginWritePacket(transmitCommand, AHRS_COMMAND_POLL_LENGTH);
}
