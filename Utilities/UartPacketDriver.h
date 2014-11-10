//-----------------------------------------------------------------------------
// XBeeDriver.h
//
// Handles packet communication through a UART port and using DMA
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------
#define UART_INTERRUPT_SIZE 8
#define UART_INTERRUPT_BUFFER_SIZE 128
#define UART_BUFFER_SIZE 256
#define UART_MAX_PACKET_SIZE 256
#define UART_PARTIAL_TIMEOUT 0.01
#define UART_COMMAND_TIMEOUT 20
//-----------------------------------------------------------------------------
// Different states
//-----------------------------------------------------------------------------
#define STATE_NONE 1
#define STATE_HEADER_RECEIVED 2
#define STATE_LENGTH_RECEIVED 3

namespace Andromeda
{
	class UartPacketDriver: public InterruptTemplate
	{
	    protected:
			char _rxBuffer0[UART_INTERRUPT_BUFFER_SIZE];
			char _rxBuffer1[UART_INTERRUPT_BUFFER_SIZE];
			short _rxRead;	//the amount of bytes already read from the buffers
			
			char _rxBuffer[UART_BUFFER_SIZE];
			char _txBuffer[UART_BUFFER_SIZE];
			char _packetBuffer[UART_BUFFER_SIZE];
			
			short _txFull;  //the bytes which are filled in the tx buffer
			short _rxFull;  //the bytes that are full in the rx buffer
			short _lastTx; //the last amount of bytes the went out through the tx;
			short _totalPacketLength; //the total length of the packet so far

			unsigned long _lastRxTicks;
			
			//registers that hold information about transferred bytes
			unsigned short _txCount;
			
			DMA_Stream_TypeDef *_txStream;
			DMA_Stream_TypeDef *_rxStream;
                        USART_TypeDef *_usart;
			const uint32_t _txTcFlag;
			const uint32_t _rxTcFlag;
			
			virtual int findPackageHeader(char *buffer, short length) = 0;
			virtual int findPackageLength(char *buffer, short length) = 0;
			void onEndTx();
			void onEndRx();
			//void onByteReceived();
			virtual bool readPacket(char *packetBuffer, short &lengthOut);

			bool isTransmitting();
            

	    public:
            UartPacketDriver(DMA_Stream_TypeDef *txStream, DMA_Stream_TypeDef *rxStream,const uint32_t txTcFlag, const uint32_t rxTcFlag, USART_TypeDef *usart);
            bool beginWritePacket(char *data, short length);
            bool readWrite(char *data, short length, char *receiveBuffer, char receiveLength);


	};
}