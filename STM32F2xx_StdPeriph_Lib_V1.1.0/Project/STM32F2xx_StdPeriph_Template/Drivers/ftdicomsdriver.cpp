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

    //configure the DMA interrupts
    configureInterrupts();
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
    RCCSetClock(m_CtsPin.m_pGpio,m_CtsPin.m_nGpioClk,ENABLE);
    RCCSetClock(m_RtsPin.m_pGpio,m_RtsPin.m_nGpioClk,ENABLE);

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

    GPIO_InitStructure.GPIO_Pin = m_CtsPin.m_nPin;
    GPIO_Init(m_CtsPin.m_pGpio, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = m_RtsPin.m_nPin;
    GPIO_Init(m_RtsPin.m_pGpio, &GPIO_InitStructure);

    // Connect USART pins to AF
    GPIO_PinAFConfig(m_TxPin.m_pGpio, m_TxPin.m_nPinSource, m_TxPin.m_nPinAF);
    GPIO_PinAFConfig(m_RxPin.m_pGpio, m_RxPin.m_nPinSource, m_RxPin.m_nPinAF);
    //GPIO_PinAFConfig(m_CtsPin.m_pGpio, m_CtsPin.m_nPinSource, m_CtsPin.m_nPinAF);
    //GPIO_PinAFConfig(m_RtsPin.m_pGpio, m_RtsPin.m_nPinSource, m_RtsPin.m_nPinAF);

    // Enable the USART OverSampling by 8
    USART_OverSampling8Cmd(FTDI_USART, ENABLE);

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

//-----------------------------------------------------------------------------
// Polls the xbee parameter without waiting for a reply
//-----------------------------------------------------------------------------
//bool FtdiComsDriver::apiPollParameterValue(const char* parameter)
//{
//    //add the identifier and frameID counts to the length
//    short length = 4;
//    char buffer[5];
//    buffer[0] = FTDI_PACKET_DELIMITER;
//    buffer[1] = int16MSB(length);
//    buffer[2] = int16LSB(length);
//    buffer[3] = FTDI_APICMD_ATCMD;
//    buffer[4] = 0x52;

//    //and now copy the data to the packet and also the checksum
//    apiCopyPacketData(buffer,5,parameter, 2);


//    //now send the packet using the dma
//    apiSendPacket();
//}

//-----------------------------------------------------------------------------
// Returns the value for a given parameter from the XBee
//-----------------------------------------------------------------------------
//bool FtdiComsDriver::apiGetParameterValue(const char* parameter,char *value,short &valueLength)
//{
//    bool bReturn = false;
//    apiPollParameterValue(parameter);
//    Delay(1);

//    //and now wait until we receive a packet (this function blocks until either
//    //timeout is reached or packet is received
//    char recBuffer[32];
//    short receiveLength;
//    setTimeout(FTDI_COMMAND_TIMEOUT);
//    while(readPacket(recBuffer,receiveLength) == false && isTimedOut() == false)
//    {}

//    //now if there was no timeout, see if we received a reply
//    if( isTimedOut() == false )
//    {
//        if( recBuffer[FTDI_ATSTATUS_BYTE_OFFSET] == FTDI_API_STATUS_OK )
//        {
//            //then we got a proper response and we can copy the value into the response
//            valueLength = receiveLength-(FTDI_ATDATA_BYTE_OFFSET+1);
//            if( valueLength <= MAX_PACKET_SIZE && valueLength > 0)
//            {
//                memmove(value, recBuffer+FTDI_ATDATA_BYTE_OFFSET, valueLength);
//                bReturn = true;
//            }
//        }
//    }
//    return bReturn;
//}

//-----------------------------------------------------------------------------
// Sends the API packet given buffers. Accepts two sets of byte pointer. Will escape
// characters from the second byte and calculate the checksum from the 4th byte
// @data1 - pointer to the first byte array
// @length1 - length of the first byte array
// @data2 - pointer to the second byte array. This can be NULL
// @length2 - lenght of the second byte array. Can be 0
//-----------------------------------------------------------------------------
void FtdiComsDriver::apiCopyPacketData(const char *data1, const short length1, const char * data2, const short length2)
{
//    int packetCounter = 0;
//    char checksum = 0;

//    //copy the first byte without any escaping or checksum
//    _packetBuffer[0] = data1[0];
//    packetCounter++;

//    //copy the data1 array
//    if( data1 != NULL )
//    {
//        for( int i = 1 ; i < length1 ; i++ )
//        {
//            char val = data1[i];
//            //increment checksum if we are at the correct byte
//            if( packetCounter >= FTDI_CHECKSUM_START_INDEX/*3*/)
//                checksum += val;

//            //escape the character if required.
////				if( val == FTDI_PACKET_DELIMITER ||
////					val == FTDI_PACKET_ESCAPE ||
////					val == FTDI_PACKET_XON ||
////					val == FTDI_PACKET_XOFF )
////				{
////					//insert an escape character
////					_packetBuffer[packetCounter] = FTDI_PACKET_ESCAPE;
////					packetCounter++;
////
////					//then XOR the value with 0x20 and add it in
////					_packetBuffer[packetCounter] = val^FTDI_PACKET_XOR;
////				}else
//                _packetBuffer[packetCounter] = val;

//            packetCounter++;
//        }
//    }

//    //copy the data2 array if possible
//    if( data2 != NULL )
//    {
//        for( int i = 0 ; i < length2 ; i++ )
//        {
//            char val = data2[i];
//            //increment checksum if we are at the correct byte
//            if( packetCounter >= FTDI_CHECKSUM_START_INDEX/*3*/)
//                checksum += val;

//            //escape the character if required.
////				if( val == FTDI_PACKET_DELIMITER ||
////					val == FTDI_PACKET_ESCAPE ||
////					val == FTDI_PACKET_XON ||
////					val == FTDI_PACKET_XOFF )
////				{
////					//insert an escape character
////					_packetBuffer[packetCounter] = FTDI_PACKET_ESCAPE;
////					packetCounter++;
////
////					//then XOR the value with 0x20 and add it in
////					_packetBuffer[packetCounter] = val^FTDI_PACKET_XOR;
////				}else
//                _packetBuffer[packetCounter] = val;

//            packetCounter++;
//        }
//    }

//    //set the checksum byte. The checksum requires all bytes
//    //to be added keeping the lower 8 and then subtracted from 0xFF
//    _packetBuffer[packetCounter] = 0xFF - checksum;

//    packetCounter++;

//    //set the total packet length
//    _totalPacketLength = packetCounter;
}

//-----------------------------------------------------------------------------
// Sends a prepared packet. apiPreparePacket must have already been called
// for this function to work.
//-----------------------------------------------------------------------------
//void FtdiComsDriver::apiSendPacket()
//{
//    beginWritePacket(_packetBuffer,_totalPacketLength);

//}

//-----------------------------------------------------------------------------
// Sends a tx packet asynchronously through the queue using 16 bit addressing.
// @address - the 16 bit (2 byte) address of the recipient module
// @data - pointer to the bytes that will get copied into the TX packet
// @dataLength - the length of data to be sent
// @disableAck - sets option bits so that the Xbee does not reply with an ack packet
// @broadcastPanID - broadcasts the tx packet on all panIDs
//-----------------------------------------------------------------------------
//void FtdiComsDriver::apiSendTxPacket16(short address,char *data,short dataLength,bool disableAck /*=false*/, bool broadcastPanID /*=false*/)
//{
//    //add the identifier ,frameID, address and option bit counts to the length
//    char buffer[8];
//    short length = dataLength + 5;
//    buffer[0] = FTDI_PACKET_DELIMITER;
//    buffer[1] = int16MSB(length);
//    buffer[2] = int16LSB(length);
//    buffer[3] = FTDI_APICMD_TX16;
//    buffer[4] = 0;	//set frameID to 0 to disable the response packet
//    buffer[5] = int16MSB(address);
//    buffer[6] = int16MSB(address);
//    if( disableAck == true )
//        buffer[7] = FTDI_API_DISABLE_ACK /*0x01*/;
//    else if( broadcastPanID == true )
//        buffer[7] = FTDI_API_BROADCAST_PAN_ID /*0x04*/;
//    else
//        buffer[7] = 0;

//    //and now copy the data to the packet and also the checksum
//    apiCopyPacketData(buffer,8,data,dataLength);

//    //and we can now send the data
//    apiSendPacket();
//}

//-----------------------------------------------------------------------------
// This function checks to see if there is a package received and if yes, will
// return the packet. It will return true if a packet is found.
// @packetBuffer - the buffer passed in to write the packet to
// @lengthOut - the length of the packet if available
//-----------------------------------------------------------------------------
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

    /***************************************************/
    /*             End Critical Section                */
    /***************************************************/
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

//-----------------------------------------------------------------------------
// This function is called externally to get data from the Xbee. It will check
// to see if any RX packets are receive and if they have been, it will return
// the data and address to the caller
//-----------------------------------------------------------------------------
//bool FtdiComsDriver::processPacket(char *packetBuffer, short &lengthOut, short &sender, char &rssi)
//{
//    short reg = 0;
//    //internally see if we can read a packet
//    char buffer[MAX_PACKET_SIZE];
//    short length;
//    //if we haven't got anything simply return false
//    if( readPacket(buffer, length ) == false )
//        return false;

//    //otherwise, see what kind of packet this was
//    switch( buffer[FTDI_ATIDENT_BYTE_OFFSET] )
//    {
//        case FTDI_APICMD_ATCMD_REP:
//            //then this is a reply to an AT command, we must figure out
//            //which register it was
//            if( buffer[FTDI_ATCMD_BYTE_OFFSET] == 'A' && buffer[FTDI_ATCMD_BYTE_OFFSET+1] == 'I' )
//            {
//                //if the new value is 0 and the old one wasn't, then we need to reset MY
//                //as it gets set to 0xFFFE and puts the module in 64 bit addressing mode
//                if( buffer[FTDI_ATDATA_BYTE_OFFSET] == FTDI_ASSOCIATION_OK && _regAI != FTDI_ASSOCIATION_OK )
//                {
//                    apiSetParameterValue("MY",(char *)(&_regMY),2,false);
//                }

//                _regAI = buffer[FTDI_ATDATA_BYTE_OFFSET];
//            }
//            break;

//        case FTDI_APICMD_RX16:
//            //we have receive a packet. First copy the address, then RSSI and then data
//            sender = (buffer[FTDI_RXADDR_BYTE_OFFSET] << 8) | buffer[FTDI_RXADDR_BYTE_OFFSET+ 1];
//            rssi = *((char*)&buffer[FTDI_RXRSSI_BYTE_OFFSET]);
//            //then length of data is the length of the packet - the extra bytes
//            lengthOut = length - 9;
//            memmove( packetBuffer,buffer+FTDI_RXDATA_BYTE_OFFSET, lengthOut);
//            return true;
//            break;
//    }

//    return false;
//}
