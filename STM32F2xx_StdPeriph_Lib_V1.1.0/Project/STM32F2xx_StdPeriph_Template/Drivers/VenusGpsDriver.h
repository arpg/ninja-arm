//-----------------------------------------------------------------------------
// VenusGpsDriver.h
//
// Contains the implementation of the UART driver to drive the Venus GPS chip
// from SkyTraq
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// The UART port definitions used for the communciation
//-----------------------------------------------------------------------------
#define VENUS_USART                     USART3
#define VENUS_CLK                       RCC_APB1Periph_USART3
#define VENUS_CLK_INIT                  RCC_APB1PeriphClockCmd
#define VENUS_IRQn                      USART3_IRQn
#define VENUS_IRQHandler                USART3_IRQHandler

#define VENUS_TX_PIN                    GPIO_Pin_10                
#define VENUS_TX_GPIO_PORT              GPIOC                       
#define VENUS_TX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define VENUS_TX_SOURCE                 GPIO_PinSource10
#define VENUS_TX_AF                     GPIO_AF_USART3

#define VENUS_RX_PIN                    GPIO_Pin_11                
#define VENUS_RX_GPIO_PORT              GPIOC                    
#define VENUS_RX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define VENUS_RX_SOURCE                 GPIO_PinSource11
#define VENUS_RX_AF                     GPIO_AF_USART3

#define VENUS_RESET_PIN                 GPIO_Pin_12                
#define VENUS_RESET_GPIO_PORT           GPIOC                    
#define VENUS_RESET_GPIO_CLK            RCC_AHB1Periph_GPIOC

//-----------------------------------------------------------------------------
// The UART port DMA definitions
//-----------------------------------------------------------------------------
#define VENUS_DR_ADDRESS                &VENUS_USART->DR		//this is the address of the register that the DMA uses 

#define VENUS_DMA                       DMA1
#define VENUS_DMAx_CLK                  RCC_AHB1Periph_DMA1
   
#define VENUS_TX_DMA_CHANNEL            DMA_Channel_4
#define VENUS_TX_DMA_STREAM             DMA1_Stream3
#define VENUS_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
#define VENUS_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
#define VENUS_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
#define VENUS_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
#define VENUS_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
            
#define VENUS_RX_DMA_CHANNEL            DMA_Channel_4
#define VENUS_RX_DMA_STREAM             DMA1_Stream1
#define VENUS_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
#define VENUS_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
#define VENUS_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
#define VENUS_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
#define VENUS_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

#define VENUS_DMA_TX_IRQn               DMA1_Stream3_IRQn
#define VENUS_DMA_RX_IRQn               DMA1_Stream1_IRQn
#define VENUS_DMA_TX_IRQHandler         DMA1_Stream3_IRQHandler
#define VENUS_DMA_RX_IRQHandler         DMA1_Stream1_IRQHandler 

#define VENUS_BAUD			9600

namespace Andromeda
{
    class VenusGpsDriver: public UartPacketDriver
    {
	protected:
	virtual void onInterruptDma1Stream1();
	virtual void onInterruptDma1Stream3();

	void configureUart();
	void configureDevice();
	void configureInterrupts();


	public:
	VenusGpsDriver();
	void initialise();
	virtual int findPackageHeader(char *buffer, short length);
	virtual int findPackageLength(char *buffer, short length);
	virtual bool readPacket(char *packetBuffer, short &lengthOut);
    };
}