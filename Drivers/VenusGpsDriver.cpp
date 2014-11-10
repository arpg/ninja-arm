//-----------------------------------------------------------------------------
// VenusGpsDriver.h
//
// Contains the implementation of the UART driver to drive the Venus GPS chip
// from SkyTraq
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------
#include "main.h"
#include "interruptTemplate.h"
#include "uartpacketdriver.h"
#include "VenusGpsDriver.h"

using namespace Andromeda;

VenusGpsDriver::VenusGpsDriver() : UartPacketDriver(VENUS_TX_DMA_STREAM,VENUS_RX_DMA_STREAM,VENUS_TX_DMA_FLAG_TCIF,VENUS_RX_DMA_FLAG_TCIF, 0)
{

}

//-----------------------------------------------------------------------------
// Sets the UART port that is to be used for the functions of this driver and also
// sets up the appropriate interrupts and also the device itself
//-----------------------------------------------------------------------------
void VenusGpsDriver::initialise()
{
    //configure the UART 
    configureUart();
    
    //configure the DMA interrupts
    configureInterrupts();
    
    //and then configure the device for operation
    configureDevice();
}

//-----------------------------------------------------------------------------
// Sets up the UART for initial communication with the module. Also sets up the
// DMA for both Tx and Rx channels.
//-----------------------------------------------------------------------------
void VenusGpsDriver::configureUart()
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    
    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(VENUS_TX_GPIO_CLK | VENUS_RX_GPIO_CLK, ENABLE);
    
    // Enable USART clock 
    VENUS_CLK_INIT(VENUS_CLK, ENABLE);
    
    // Reset UART IP 
    RCC_APB1PeriphResetCmd(VENUS_CLK, ENABLE);
    RCC_APB1PeriphResetCmd(VENUS_CLK, DISABLE);
    
    // Connect USART pins to AF7
    GPIO_PinAFConfig(VENUS_TX_GPIO_PORT, VENUS_TX_SOURCE, VENUS_TX_AF);
    GPIO_PinAFConfig(VENUS_RX_GPIO_PORT, VENUS_RX_SOURCE, VENUS_RX_AF);
    
    
    // Configure USART Tx and Rx as alternate function push-pull
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    
    GPIO_InitStructure.GPIO_Pin = VENUS_TX_PIN;
    GPIO_Init(VENUS_TX_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = VENUS_RX_PIN;
    GPIO_Init(VENUS_RX_GPIO_PORT, &GPIO_InitStructure);

    //initialize the RESET pin and then RESET the GPS
    GPIO_InitStructure.GPIO_Pin = VENUS_RESET_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(VENUS_RESET_GPIO_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(VENUS_RESET_GPIO_PORT,VENUS_RESET_PIN);
    Delay(1);
    GPIO_SetBits(VENUS_RESET_GPIO_PORT,VENUS_RESET_PIN);


    
    // Enable the USART OverSampling by 8 
	USART_OverSampling8Cmd(VENUS_USART, ENABLE);  
    
    //the baud rate of the module has to be pre-set and saved
    //to the baud rate specified here.
    USART_InitStructure.USART_BaudRate = VENUS_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(VENUS_USART, &USART_InitStructure);
    
    //configure the DMA clock
    RCC_AHB1PeriphClockCmd(VENUS_DMAx_CLK, ENABLE);  
    
    //setup the DMA initialization structure
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)VENUS_DR_ADDRESS;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    //DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    
    //initialise the RX DMA with the RX0 buffer
    DMA_DeInit(VENUS_RX_DMA_STREAM);
    //wait for the EN bit to go low
    while(DMA_GetCmdStatus(VENUS_RX_DMA_STREAM) == ENABLE)
    {}
    
    DMA_InitStructure.DMA_Channel = VENUS_RX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_rxBuffer0;
    DMA_InitStructure.DMA_BufferSize = (uint16_t)UART_INTERRUPT_SIZE;
    DMA_Init(VENUS_RX_DMA_STREAM, &DMA_InitStructure);
    
    //initialise the TX DMA with the TX buffer
    DMA_DeInit(VENUS_TX_DMA_STREAM);
    //wait for the EN bit to go low
    while(DMA_GetCmdStatus(VENUS_TX_DMA_STREAM) == ENABLE)
    {}
    DMA_InitStructure.DMA_Channel = VENUS_TX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_txBuffer;
    DMA_InitStructure.DMA_BufferSize = 0; //no TX data to begin with
    DMA_Init(VENUS_TX_DMA_STREAM, &DMA_InitStructure);
    
    //disable both DMAs
    DMA_Cmd(VENUS_TX_DMA_STREAM, DISABLE);
    DMA_Cmd(VENUS_RX_DMA_STREAM, DISABLE);
    
    //set up the DMA double buffering for RX by setting the second address to buffer2 
    //and setting the active DMA memory to buffer1
    DMA_DoubleBufferModeConfig(VENUS_RX_DMA_STREAM,(uint32_t)_rxBuffer1,DMA_Memory_0);
    
    //then enable double buffer mode
    DMA_DoubleBufferModeCmd(VENUS_RX_DMA_STREAM, ENABLE);
    
    //enable the UART
    USART_Cmd(VENUS_USART, ENABLE);
    
    //enable the DMA IRQ requests for the UART for both
    //rx and tx
    USART_DMACmd(VENUS_USART, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(VENUS_USART, USART_DMAReq_Tx, ENABLE);
    
    USART_ClearFlag(VENUS_USART, USART_FLAG_TC);
    
    DMA_Cmd(VENUS_RX_DMA_STREAM, ENABLE);
    
    //enable TC interrupt for the RX stream
    DMA_ITConfig(VENUS_RX_DMA_STREAM,DMA_IT_TC,ENABLE);
}

//-----------------------------------------------------------------------------
// Sets up the DMA interrupts and anything else that's required by the Xbee
//-----------------------------------------------------------------------------
void VenusGpsDriver::configureInterrupts()
{
    //and now register this class to receive the RX and TX interrupts
    InterruptTemplate::registerForInterrupt(DMA1STREAM1_INTERRUPT_HANDLER,this);
    InterruptTemplate::registerForInterrupt(DMA1STREAM3_INTERRUPT_HANDLER,this);
    
    NVIC_InitTypeDef   NVIC_InitStructure;
    // enable the RX Dma interrupts
    NVIC_InitStructure.NVIC_IRQChannel = VENUS_DMA_RX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    //  enable the TX Dma interrupts
    NVIC_InitStructure.NVIC_IRQChannel = VENUS_DMA_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//-----------------------------------------------------------------------------
// Events called by the interrupt manager
//-----------------------------------------------------------------------------
void VenusGpsDriver::onInterruptDma1Stream1()
{
    onEndRx();
}

void VenusGpsDriver::onInterruptDma1Stream3()
{
    onEndTx();
}

//-----------------------------------------------------------------------------
// finds the package header in the given byte stream
//-----------------------------------------------------------------------------
int VenusGpsDriver::findPackageHeader(char *buffer, short length)
{
    return -1;
}

//-----------------------------------------------------------------------------
// finds the package length in the given byte stream
//-----------------------------------------------------------------------------
int VenusGpsDriver::findPackageLength(char *buffer, short length)
{
    return -1;
}

//-----------------------------------------------------------------------------
// Configures the device for initial operation
//-----------------------------------------------------------------------------
void VenusGpsDriver::configureDevice()
{
}

//-----------------------------------------------------------------------------
// Uses the underlying packet driver to read packets from the stream and pass
// them to the called
//-----------------------------------------------------------------------------
bool VenusGpsDriver::readPacket(char *packetBuffer, short &lengthOut)
{
    return UartPacketDriver::readPacket(packetBuffer,lengthOut);
}


