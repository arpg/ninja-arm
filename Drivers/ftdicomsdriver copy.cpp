#include "ftdicomsdriver.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_tim.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_usart.h"
#include "stm32f2xx_dma.h"
#include <cstring>
#include <cmath>

FtdiComsDriver::FtdiComsDriver()
{
}

bool FtdiComsDriver::ChkChksum(CommandPacket* _data)
{
  unsigned char* DataPtr = (unsigned char*)_data;
  unsigned short int Sum = 0;

  for(int ii=0;ii<sizeof(CommandPacket)-2;ii++)
    Sum += DataPtr[ii];

  if(Sum == _data->Checksum)
    return(true);
  else
    return(false);
}
void FtdiComsDriver::AddChecksum(Transmit_CommandPacket &_data)
{
  unsigned char* DataPtr = (unsigned char*)&_data;
  unsigned short int Sum = 0;

  for(int ii=0;ii<sizeof(Transmit_CommandPacket)-2;ii++)
    Sum += DataPtr[ii];
  
  _data.Checksum = Sum;

}
//-----------------------------------------------------------------------------
// Sets the UART port that is to be used for the functions of this driver and also
// sets up the appropriate interrupts and also the device itself
//-----------------------------------------------------------------------------
void FtdiComsDriver::Initialize(GPIO_Pin txPin, GPIO_Pin rxPin, GPIO_Pin ctsPin, GPIO_Pin rtsPin)
{
    m_RtsPin = rtsPin;
    m_CtsPin = ctsPin;
    m_TxPin = txPin;
    m_RxPin = rxPin;


    //configure the transmission counters
    _rxFull = 0;
    _txFull = 0;
    _rxRead = 0;
    _lastTx = -1;
    _totalPacketLength = 0;

    //reset the byte counters
    _txCount = 0;

    //configure the UART
    configureUart();
    //my_configureUart();

    //configure the DMA interrupts
    configureInterrupts();
}

void FtdiComsDriver::my_configureUart()
{
  
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //USART3 Clock Enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //GPIOB Clock Enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); //DMA1 Clock Enable
 
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); //USART3 Tx Pin
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); //USART3 Rx Pin
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitTypeDef NVIC_InitStructure;
  
  // Enable the USART3 RX DMA Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  USART_InitTypeDef USART_InitStruct;
  USART_StructInit(&USART_InitStruct);
  USART_InitStruct.USART_BaudRate = 19200;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART3, &USART_InitStruct);
  
  USART_Cmd(USART3, ENABLE); // Enable USART3


  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  // ----- USART3 DMA Rx Stream Init -----
  DMA_DeInit(DMA1_Stream1);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Receive From Per to Mem
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RxBuffer;
  DMA_InitStructure.DMA_BufferSize = RXBUFFERSIZE;
  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable; // ??????
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  

    //wait for the EN bit to go low
    while(DMA_GetCmdStatus(FTDI_RX_DMA_STREAM) == ENABLE)
    {}

    DMA_Init(FTDI_RX_DMA_STREAM, &DMA_InitStructure);

    //initialise the TX DMA with the TX buffer
    DMA_DeInit(FTDI_TX_DMA_STREAM);
    //wait for the EN bit to go low
    while(DMA_GetCmdStatus(FTDI_TX_DMA_STREAM) == ENABLE)
    {}
    DMA_InitStructure.DMA_Channel = FTDI_TX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_txBuffer;
    DMA_InitStructure.DMA_BufferSize = 0; //no TX data to begin with
    DMA_Init(FTDI_TX_DMA_STREAM, &DMA_InitStructure);

//////////////////////////////////////////

  //disable both DMAs
    DMA_Cmd(FTDI_TX_DMA_STREAM, DISABLE);
    DMA_Cmd(FTDI_RX_DMA_STREAM, DISABLE);

    //set up the DMA double buffering for RX by setting the second address to buffer2
    //and setting the active DMA memory to buffer1
    DMA_DoubleBufferModeConfig(FTDI_RX_DMA_STREAM,(uint32_t)_rxBuffer1,DMA_Memory_0);

    //then enable double buffer mode
    DMA_DoubleBufferModeCmd(FTDI_RX_DMA_STREAM, ENABLE);

    // Enable RXNE interrupt 
    USART_ITConfig(FTDI_USART, USART_IT_RXNE, ENABLE);
    // Enable USARTx global interrupt 
    NVIC_EnableIRQ(FTDI_IRQn);

    //enable the UART
    USART_Cmd(FTDI_USART, ENABLE);

    //enable the DMA IRQ requests for the UART for both
    //rx and tx
    USART_DMACmd(FTDI_USART, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(FTDI_USART, USART_DMAReq_Tx, ENABLE);

    USART_ClearFlag(FTDI_USART, USART_FLAG_TC);

    DMA_Cmd(FTDI_RX_DMA_STREAM, ENABLE);

    //enable TC interrupt for the RX stream
    DMA_ITConfig(FTDI_RX_DMA_STREAM,DMA_IT_TC,ENABLE);
///////////////////////////////////////////


//  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
  
//  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE); // Enable USART Rx DMA Request
//  DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE); // Enable Transfer Complete Interrupt
//  DMA_Cmd(DMA1_Stream1, ENABLE); // Enable DMA Rx Stream
}

//-----------------------------------------------------------------------------
// Sets up the UART for initial communication with the module. Also sets up the
// DMA for both Tx and Rx channels.
//-----------------------------------------------------------------------------
void FtdiComsDriver::configureUart()
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    USART_StructInit(&USART_InitStructure);
    DMA_StructInit(&DMA_InitStructure);

    // Enable GPIO clock
    RCCSetClock(m_TxPin.m_pGpio,m_TxPin.m_nGpioClk,ENABLE);
    RCCSetClock(m_RxPin.m_pGpio,m_RxPin.m_nGpioClk,ENABLE);
    
    // Enable USART clock
    RCCSetClock(FTDI_USART,FTDI_USART_CLK, ENABLE);

    // Reset UART IP
    RCC_APB1PeriphResetCmd(FTDI_USART_CLK, ENABLE);
    RCC_APB1PeriphResetCmd(FTDI_USART_CLK, DISABLE);

    // Configure USART Tx and Rx as alternate function push-pull
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = m_TxPin.m_nPin;
    GPIO_Init(m_TxPin.m_pGpio, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = m_RxPin.m_nPin;
    GPIO_Init(m_RxPin.m_pGpio, &GPIO_InitStructure);

    // Connect USART pins to AF
    GPIO_PinAFConfig(m_TxPin.m_pGpio, m_TxPin.m_nPinSource, m_TxPin.m_nPinAF);
    GPIO_PinAFConfig(m_RxPin.m_pGpio, m_RxPin.m_nPinSource, m_RxPin.m_nPinAF);

    //the baud rate of the module has to be pre-set and saved
    //to the baud rate specified here.
    USART_InitStructure.USART_BaudRate = FTDI_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//USART_HardwareFlowControl_RTS_CTS;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(FTDI_USART, &USART_InitStructure);
    
    //configure the DMA clock
    RCCSetClock(FTDI_DMA,FTDI_DMAx_CLK, ENABLE);

    //setup the DMA initialization structure
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)FTDI_DR_ADDRESS;
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
    // Here only the unchanged parameters of the DMA initialization structure are
    //configured. During the program operation, the DMA will be configured with
    //different parameters according to the operation phase 

    //initialise the RX DMA with the RX0 buffer
    DMA_DeInit(FTDI_RX_DMA_STREAM);
    //wait for the EN bit to go low
    while(DMA_GetCmdStatus(FTDI_RX_DMA_STREAM) == ENABLE)
    {}

    DMA_InitStructure.DMA_Channel = FTDI_RX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_rxBuffer0;
    DMA_InitStructure.DMA_BufferSize = (uint16_t)FTDI_INTERRUPT_SIZE;
    DMA_Init(FTDI_RX_DMA_STREAM, &DMA_InitStructure);

    //initialise the TX DMA with the TX buffer
    DMA_DeInit(FTDI_TX_DMA_STREAM);
    //wait for the EN bit to go low
    while(DMA_GetCmdStatus(FTDI_TX_DMA_STREAM) == ENABLE)
    {}
    DMA_InitStructure.DMA_Channel = FTDI_TX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)_txBuffer;
    DMA_InitStructure.DMA_BufferSize = 0; //no TX data to begin with
    DMA_Init(FTDI_TX_DMA_STREAM, &DMA_InitStructure);

    //disable both DMAs
    DMA_Cmd(FTDI_TX_DMA_STREAM, DISABLE);
    DMA_Cmd(FTDI_RX_DMA_STREAM, DISABLE);

    //set up the DMA double buffering for RX by setting the second address to buffer2
    //and setting the active DMA memory to buffer1
    DMA_DoubleBufferModeConfig(FTDI_RX_DMA_STREAM,(uint32_t)_rxBuffer1,DMA_Memory_0);

    //then enable double buffer mode
    DMA_DoubleBufferModeCmd(FTDI_RX_DMA_STREAM, ENABLE);

    /* Enable RXNE interrupt */
    USART_ITConfig(FTDI_USART, USART_IT_RXNE, ENABLE);
    /* Enable USARTx global interrupt */
    NVIC_EnableIRQ(FTDI_IRQn);


    //enable the UART
    USART_Cmd(FTDI_USART, ENABLE);

    //enable the DMA IRQ requests for the UART for both
    //rx and tx
    USART_DMACmd(FTDI_USART, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(FTDI_USART, USART_DMAReq_Tx, ENABLE);

    USART_ClearFlag(FTDI_USART, USART_FLAG_TC);

    DMA_Cmd(FTDI_RX_DMA_STREAM, ENABLE);

    //enable TC interrupt for the RX stream
    DMA_ITConfig(FTDI_RX_DMA_STREAM,DMA_IT_TC,ENABLE);
}

//-----------------------------------------------------------------------------
// Sets up the DMA interrupts and anything else that's required by the Xbee
//-----------------------------------------------------------------------------
void FtdiComsDriver::configureInterrupts()
{
    //and now register this class to receive the RX and TX interrupts
    InterruptTemplate::registerForInterrupt(DMA1STREAM1_INTERRUPT_HANDLER,this);
    InterruptTemplate::registerForInterrupt(DMA1STREAM3_INTERRUPT_HANDLER,this);

    NVIC_InitTypeDef NVIC_InitStructure;

     // enable the RX Dma interrupts
    NVIC_InitStructure.NVIC_IRQChannel = FTDI_DMA_RX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //  enable the TX Dma interrupts
    NVIC_InitStructure.NVIC_IRQChannel = FTDI_DMA_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//-----------------------------------------------------------------------------
// RX DMA interrupt handler
//-----------------------------------------------------------------------------
void FtdiComsDriver::onEndRx()
{
    int i = _rxRead;
    int j = _rxFull;
    //figure out which buffer is currently active
    uint32_t activeMemory = DMA_GetCurrentMemoryTarget(FTDI_RX_DMA_STREAM);

    unsigned char *fullBuffer;
    //first detect which buffer is full
    if( activeMemory == 1)
    {
        fullBuffer = _rxBuffer0 + _rxRead;
        //reconfigure the memory pointer
        //DMA_MemoryTargetConfig(FTDI_RX_DMA_STREAM,(uint32_t)_rxBuffer0,DMA_Memory_0);
    }else
    {
        fullBuffer = _rxBuffer1 + _rxRead;
        //reconfigure the memory pointer
        //DMA_MemoryTargetConfig(FTDI_RX_DMA_STREAM,(uint32_t)_rxBuffer1,DMA_Memory_1);
    }

    //now copy the data to the main rx buffer if there is space
    if( (_rxFull + FTDI_INTERRUPT_SIZE) < FTDI_UART_RX_BUFFER_SIZE)
    {
        //if bytes have already been read previously, do not copy them into the buffer
        //only copy what's been newly received
        memmove( _rxBuffer+_rxFull, fullBuffer, FTDI_INTERRUPT_SIZE-_rxRead);
        _rxFull += FTDI_INTERRUPT_SIZE-_rxRead;
    }else
    {
        //we need to discard the buffer. It is full
        _rxFull = 0;
    }


    //reset the read count
    _rxRead = 0;

    //clear the TC flag
    DMA_ClearFlag(FTDI_RX_DMA_STREAM, FTDI_RX_DMA_FLAG_TCIF);
    //activeMemory = DMA_GetCurrentMemoryTarget(FTDI_RX_DMA_STREAM);
}
//////////////////////////////////////////////////////////////////////////////////
void FtdiComsDriver::onInterruptDma1Stream1()
{
    onEndRx();
}

//-----------------------------------------------------------------------------
// TX DMA interrupt handler
//-----------------------------------------------------------------------------
void FtdiComsDriver::onEndTx()
{
    // Clear any pending flag on Rx Stream
    DMA_ClearFlag(FTDI_TX_DMA_STREAM, FTDI_TX_DMA_FLAG_TCIF);

    //if we did actually transmit some shit, then clear them from the TX buffer
    if( _lastTx != -1 )
    {
        _txFull -= _lastTx;
        if( _txFull < 0 )
            _txFull = 0;
        //and we must move everything back
        memmove(_txBuffer, _txBuffer+_lastTx, _txFull);
    }

    //if there is no more data to send, disable the ENDTX interrupt
    if( _txFull <= 0 )
    {
        DMA_Cmd(FTDI_TX_DMA_STREAM, DISABLE);
        DMA_ITConfig(FTDI_TX_DMA_STREAM,DMA_IT_TC,DISABLE);
        _lastTx = -1;
        return;
    }

     /* Clear the TC bit in the SR register by writing 0 to it */
    DMA_SetCurrDataCounter(FTDI_TX_DMA_STREAM,_txFull);
    DMA_Cmd(FTDI_TX_DMA_STREAM, ENABLE);
    _txCount += _txFull;
    _lastTx = _txFull;

    //and now we can enabled the TXBUFE interrupt
    DMA_ITConfig(FTDI_TX_DMA_STREAM,DMA_IT_TC,ENABLE);
}
//////////////////////////////////////////////////////////////////////////////////
void FtdiComsDriver::onInterruptDma1Stream3()
{
     onEndTx();
}

bool FtdiComsDriver::ReadPacket(unsigned char *packetBuffer, short &lengthOut)
{
    int i = _rxRead;
    int j = _rxFull;
    //this is a critical section so we start by disabling interrupts
    /***************************************************/
    /*             Start Critical Section              */
    /***************************************************/
    disableInterrupts();

    int curRx = DMA_GetCurrDataCounter(FTDI_RX_DMA_STREAM);

    //if this value is not equal to the MAX value, this means that
    //there's been a reception but not enough to trigger an interrupt, so we manually
    //take the data out
/*    if( curRx != (FTDI_INTERRUPT_SIZE-_rxRead) )
    {
       curRx = (FTDI_INTERRUPT_SIZE-_rxRead) - curRx;
       if(abs(curRx) > FTDI_UART_RX_BUFFER_SIZE){
            int i = 0;
        }

        if(abs(_rxRead) > FTDI_UART_RX_BUFFER_SIZE){
            int i = 0;
        }

        //figure out which buffer is currently active
        uint32_t activeMemory = DMA_GetCurrentMemoryTarget(FTDI_RX_DMA_STREAM);

        unsigned char* fullBuffer;
        //first detect which buffer is full
        if( activeMemory == 0)
        {
            fullBuffer = _rxBuffer0 + _rxRead;
        }else
        {
            fullBuffer = _rxBuffer1 + _rxRead;
        }

        //now copy the data to the main rx buffer if there is space
        if( (_rxFull + curRx) < FTDI_UART_RX_BUFFER_SIZE)
        {
            memmove( _rxBuffer+_rxFull, fullBuffer, curRx);
            _rxFull += curRx;
        }else
        {
            //we need to discard the buffer. It is full
            _rxFull = 0;
        }

        //increase the read count
        _rxRead += curRx;        
    }
*/

    //then try and find a packet header
    int startIndex = findPackageHeader(_rxBuffer, _rxFull);
    if( startIndex == -1 )
    {
        //if no header was found, exit
        //_rxFull = 0;
        enableInterrupts();
        return 0;
    }

    //if a header is found, try and get the length
    int capturedLength = _rxFull - startIndex;
    unsigned char *packetPointer = &_rxBuffer[startIndex];
    int packetLength = findPackageLength(packetPointer, capturedLength);

    if( packetLength == -1 )
    {
        //if no header was found, reset the RX buffer and go back to 0
        //_rxFull = 0;
        enableInterrupts();
        return 0;
    }

    //now we have to remove whatever it is before this packet
    if( startIndex != 0 )
    {
        memmove(_rxBuffer, packetPointer, capturedLength);
        _rxFull = capturedLength;
    }

    //the packet is now at the beginning of the buffer
    packetPointer = _rxBuffer;

    //if the packet byte hasn't been received yet, we return 0 but do not discard the buffer
    //contents
    if( packetLength == -1 || packetLength > MAX_PACKET_SIZE )
    {
        _rxFull = 0;
        enableInterrupts();
        return 0;
    }

    //if we don't have the whole thing, we must wait
    if( capturedLength < packetLength )
    {
        enableInterrupts();
        return 0;
    }

    //otherwise the whole packet is here
    lengthOut = packetLength;
    memmove(packetBuffer, packetPointer, packetLength);

    //now we have to remove the packet from the RX buffer
    int remainingLength = _rxFull-packetLength;
    memmove(_rxBuffer, _rxBuffer+packetLength, remainingLength );
    _rxFull = remainingLength ;

    /***************************************************/
    /*             End Critical Section                */
    /***************************************************/
    //and reenable the interrupts
    enableInterrupts();
    return true;
}

//-----------------------------------------------------------------------------
// Finds the start of the package using the header
//-----------------------------------------------------------------------------
int FtdiComsDriver::findPackageHeader(unsigned char *buffer, short length)
{
    for( int ii = 0 ; ii < length-1 ; ii++ )
    {
        if(buffer[ii] ==  FTDI_PACKET_DELIMITER1 &&
           buffer[ii+1] == FTDI_PACKET_DELIMITER2)
            return ii;
    }
    return -1;
}

//-----------------------------------------------------------------------------
// Attempts to return the package length
//-----------------------------------------------------------------------------
int FtdiComsDriver::findPackageLength(unsigned char *buffer, short length)
{
    if( length < 3 ) //since the length byte is the third bytes
        return -1;

    return (int)buffer[2];  //return the package length after the 2 delimiter bytes
}

/////////////////////////////////////////////////////////////////////////////////
bool FtdiComsDriver::beginWritePacket(char *data, short length)
{
    //this is a critical section so we start by disabling interrupts
    /***************************************************/
    /*             Start Critical Section              */
    /***************************************************/
    disableInterrupts();
    //make sure there is enough space
    if( (length + _txFull) > FTDI_UART_TX_BUFFER_SIZE )
    {
        _txFull = 0;  //if the buffer overflows, we must empty it so we can keep sending
        enableInterrupts();
        //enable the ENDTX interrupt so this stuff actually gets sent
        DMA_ITConfig(FTDI_TX_DMA_STREAM,DMA_IT_TC,ENABLE);

        return 0;
    }

    //and now copy data tot the tx buffer
    memmove(&_txBuffer[_txFull], data, length);
    _txFull += length;

    //and reenable the interrupts
    enableInterrupts();

    //check to see if we have already finished, if so jump straight to the function,
    //otherwise enable the end interrupt
    if( DMA_GetCmdStatus(FTDI_TX_DMA_STREAM) == ENABLE && DMA_GetCurrDataCounter(FTDI_TX_DMA_STREAM)!=0)
        DMA_ITConfig(FTDI_TX_DMA_STREAM,DMA_IT_TC,ENABLE);
    else
        onEndTx();

    return true;
}
