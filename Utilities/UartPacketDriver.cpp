//-----------------------------------------------------------------------------
// XBeeDriver.h
//
// Handles packet communication through a UART port and using DMA
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------

#include "main.h"
#include "interruptTemplate.h"
#include "uartpacketdriver.h"

using namespace Andromeda;

//-----------------------------------------------------------------------------
// Prepares all the variables required to start the packet driver
//-----------------------------------------------------------------------------
UartPacketDriver::UartPacketDriver(DMA_Stream_TypeDef *txStream, DMA_Stream_TypeDef *rxStream,const uint32_t txTcFlag, const uint32_t rxTcFlag, USART_TypeDef *usart) : 
_txStream(txStream), _rxStream(rxStream), _txTcFlag(txTcFlag), _rxTcFlag(rxTcFlag), _usart(usart)
{
    //configure the transmission counters
    _rxFull = 0;
    _txFull = 0;
    _rxRead = 0;
    _lastTx = -1;
    _totalPacketLength = 0;
    
    //reset the byte counters
    _txCount = 0;

    _lastRxTicks= 0;

    DMA_ClearFlag(_txStream, _rxTcFlag);
	DMA_ClearFlag(_rxStream, _rxTcFlag);
}

//-----------------------------------------------------------------------------
// Should be called by the derived class when a TX completed event occurs on the DMA
//-----------------------------------------------------------------------------
void UartPacketDriver::onEndTx()
{
    // Clear any pending flag on Rx Stream  
    DMA_ClearFlag(_txStream, _txTcFlag);
    
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
	    DMA_Cmd(_txStream, DISABLE);
	    DMA_ITConfig(_txStream,DMA_IT_TC,DISABLE);
	    _lastTx = -1;
	    return;
    }
    
    /* Clear the TC bit in the SR register by writing 0 to it */
    DMA_SetCurrDataCounter(_txStream,_txFull);
    DMA_Cmd(_txStream, ENABLE);
    _txCount += _txFull;
    _lastTx = _txFull;
    
    //and now we can enabled the TXBUFE interrupt
    DMA_ITConfig(_txStream,DMA_IT_TC,ENABLE);

    //reset the ticks
    _lastRxTicks = getTicks();

}

//-----------------------------------------------------------------------------
// Should be called by the derived class when a RX completed event occurs on the DMA
//-----------------------------------------------------------------------------
void UartPacketDriver::onEndRx()
{
    int i = _rxRead;
    int j = _rxFull;
    //figure out which buffer is currently active
    uint32_t activeMemory = DMA_GetCurrentMemoryTarget(_rxStream);
	//calculates the actual number of bytes read
	char bytesRead = UART_INTERRUPT_SIZE-_rxRead;
    
    char *fullBuffer;
    //first detect which buffer is full
    if( activeMemory == 1)
    {
	fullBuffer = _rxBuffer0 + _rxRead;
    }else
    {
	fullBuffer = _rxBuffer1 + _rxRead;
    }
    
    //now copy the data to the main rx buffer if there is space
    if( (_rxFull + UART_INTERRUPT_SIZE) < UART_BUFFER_SIZE)
    {
		//if bytes have already been read previously, do not copy them into the buffer
		//only copy what's been newly received
		memmove( _rxBuffer+_rxFull, fullBuffer, bytesRead);
		_rxFull += bytesRead;
    }else
    {
		//we need to discard the buffer. It is full
		_rxFull = 0;
    }
    //reset the read count
    _rxRead = 0;

	//reset the ticks
    _lastRxTicks = getTicks();
    
    //clear the TC flag
    DMA_ClearFlag(_rxStream, _rxTcFlag);

    i = _rxRead;
    j = _rxFull;
}

//-----------------------------------------------------------------------------
// This function checks to see if there is a package received and if yes, will
// return the packet. It will return true if a packet is found.
// @packetBuffer - the buffer passed in to write the packet to
// @lengthOut - the length of the packet if available
//-----------------------------------------------------------------------------
bool UartPacketDriver::readPacket(char *packetBuffer, short &lengthOut)
{
    int i = _rxRead;
    int j = _rxFull;
    //this is a critical section so we start by disabling interrupts
    /***************************************************/
    /*             Start Critical Section              */
    /***************************************************/
    disableInterrupts();

	unsigned long ticks = getTicks();
	float dT = getTimeSpan(ticks,_lastRxTicks);
	if( dT > UART_PARTIAL_TIMEOUT)
	{
		//then there has been a period of inactivity and we have to download
		//whatever is currently left and reset everything
        int curRx = DMA_GetCurrDataCounter(_rxStream);
	
		//if this value is not equal to the MAX value, this means that 
		//there's been a reception but not enough to trigger an interrupt, so we manually
		//take the data out
		if( curRx != (UART_INTERRUPT_SIZE-_rxRead) )
		{
			curRx = (UART_INTERRUPT_SIZE-_rxRead) - curRx;
			if( curRx > 0 )
			{
				//figure out which buffer is currently active
				uint32_t activeMemory = DMA_GetCurrentMemoryTarget(_rxStream);
			
				char *fullBuffer;
				//first detect which buffer is full
				if( activeMemory == 0)
				{
					fullBuffer = _rxBuffer0 + _rxRead;
				}else
				{
					fullBuffer = _rxBuffer1 + _rxRead;
				}
			
				//now copy the data to the main rx buffer if there is space
				if( (_rxFull + curRx) < UART_BUFFER_SIZE)
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
		}
	}


    //then try and find a packet header
    short startIndex = findPackageHeader(_rxBuffer, _rxFull);
	if( startIndex == -1 )
	{
		//if no header was found, exit
		enableInterrupts();
		return 0;
    }

    //if a header is found, try and get the length
    short capturedLength = _rxFull - startIndex;
    char *packetPointer = &_rxBuffer[startIndex];
    short packetLength = findPackageLength(packetPointer, capturedLength);


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
    if( packetLength == -1 || packetLength > UART_MAX_PACKET_SIZE )
    {
		//only erase the contents if package is too long
		if( packetLength > UART_MAX_PACKET_SIZE )
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
    short remainingLength = _rxFull-packetLength;
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
// Writes the given data to the TX buffer and initiates the send operation if 
// possible. Otherwise the data will be sent as soon as the previous send operation
// is finished
//-----------------------------------------------------------------------------
bool UartPacketDriver::beginWritePacket(char *data, short length)
{
    //this is a critical section so we start by disabling interrupts
    /***************************************************/
    /*             Start Critical Section              */
    /***************************************************/
    disableInterrupts();
    //make sure there is enough space
    if( (length + _txFull) > UART_BUFFER_SIZE )
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
	    onEndTx();

    return true;
}

//-----------------------------------------------------------------------------
// Returns a boolean value indicating if the UART is in transmit mode
//-----------------------------------------------------------------------------
bool UartPacketDriver::isTransmitting()
{
  return (DMA_GetCmdStatus(_txStream) == ENABLE && DMA_GetCurrDataCounter(_txStream)!=0);
}

//-----------------------------------------------------------------------------
// Attempts a synchronous read/write cycle
//-----------------------------------------------------------------------------
bool UartPacketDriver::readWrite(char *data, short length, char *receiveBuffer, char receiveLength)
{
	//wait until the current TX operation is finished 
    while( DMA_GetCmdStatus(_txStream) == ENABLE && DMA_GetCurrDataCounter(_txStream)!=0)
	{}
		
    //clear UART flags 
    USART_ClearFlag(_usart,USART_FLAG_CTS|USART_FLAG_LBD|USART_FLAG_TC|USART_FLAG_RXNE);

	//copy the send stuff into the buffer
	memmove(_txBuffer,data,length);

	_rxFull = 0;

	char *temp = _rxBuffer;

	//enable the RX DMA
    DMA_Cmd(_rxStream, DISABLE);
    DMA_SetCurrDataCounter(_rxStream,UART_INTERRUPT_SIZE);
    DMA_Cmd(_rxStream, ENABLE);

	//send the stuff.
    DMA_Cmd(_txStream, DISABLE);
    DMA_SetCurrDataCounter(_txStream,length);
    DMA_ClearFlag(_txStream, _txTcFlag);
    DMA_Cmd(_txStream, ENABLE);

	//wait until we have received everything
	setTimeout(UART_COMMAND_TIMEOUT);
	while(isTimedOut() == false)
	{
		char *fullBuffer;
		int counter = UART_INTERRUPT_SIZE - DMA_GetCurrDataCounter(_rxStream);
		int total = counter +_rxFull;
		if( total >= receiveLength )
		{
			disableInterrupts();
			//then we have got everything
            uint32_t activeMemory = DMA_GetCurrentMemoryTarget(_rxStream);

            if( activeMemory == 0)
			{
				fullBuffer = _rxBuffer0;
			}else
			{
				fullBuffer = _rxBuffer1;
			}

			memmove( receiveBuffer, _rxBuffer, _rxFull);
            memmove( receiveBuffer+_rxFull , fullBuffer, counter);
			


            enableInterrupts();
			break;
		}
	}

    DMA_Cmd(_rxStream, DISABLE);
    DMA_SetCurrDataCounter(_rxStream,UART_INTERRUPT_SIZE);
    DMA_Cmd(_rxStream, ENABLE);
    _rxFull = 0;

	if( isTimedOut()  )
		return false;
	else
		return true;
}
