//-----------------------------------------------------------------------------
// AhrsModuleDriver.h
//
// This file contains the implementation of the UART driver to communicate with
// the AHRS module of the andromeda autopilot
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// The SPI port definitions for the Module
//-----------------------------------------------------------------------------
#define AHRS_SPI						SPI2
#define AHRS_SPI_IRQ					SPI2_IRQn
#define AHRS_SPI_CLK					RCC_APB1Periph_SPI2
#define AHRS_SPI_CLK_INIT				RCC_APB1PeriphClockCmd

#define AHRS_SPI_NSS_PIN                GPIO_Pin_12
#define AHRS_SPI_NSS_GPIO_PORT          GPIOB
#define AHRS_SPI_NSS_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define AHRS_SPI_NSS_SOURCE             GPIO_PinSource12
#define AHRS_SPI_NSS_AF                 GPIO_AF_SPI2

#define AHRS_SPI_SCK_PIN                GPIO_Pin_13
#define AHRS_SPI_SCK_GPIO_PORT          GPIOB
#define AHRS_SPI_SCK_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define AHRS_SPI_SCK_SOURCE             GPIO_PinSource13
#define AHRS_SPI_SCK_AF                 GPIO_AF_SPI2

#define AHRS_SPI_MISO_PIN               GPIO_Pin_14
#define AHRS_SPI_MISO_GPIO_PORT         GPIOB
#define AHRS_SPI_MISO_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define AHRS_SPI_MISO_SOURCE            GPIO_PinSource14
#define AHRS_SPI_MISO_AF                GPIO_AF_SPI2

#define AHRS_SPI_MOSI_PIN               GPIO_Pin_15
#define AHRS_SPI_MOSI_GPIO_PORT         GPIOB
#define AHRS_SPI_MOSI_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define AHRS_SPI_MOSI_SOURCE            GPIO_PinSource15
#define AHRS_SPI_MOSI_AF                GPIO_AF_SPI2

//-----------------------------------------------------------------------------
// The UART port DMA definitions
//-----------------------------------------------------------------------------
#define AHRS_DR_ADDRESS                &AHRS_SPI->DR		//this is the address of the register that the DMA uses 

#define AHRS_DMA                       DMA1
#define AHRS_DMAx_CLK                  RCC_AHB1Periph_DMA1
   
#define AHRS_TX_DMA_CHANNEL            DMA_Channel_0
#define AHRS_TX_DMA_STREAM             DMA1_Stream4
#define AHRS_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF4
            
#define AHRS_RX_DMA_CHANNEL            DMA_Channel_0
#define AHRS_RX_DMA_STREAM             DMA1_Stream3
#define AHRS_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3

#define AHRS_SPI_DMA_TX_IRQn           DMA1_Stream4_IRQn
#define AHRS_SPI_DMA_RX_IRQn           DMA1_Stream3_IRQn
#define AHRS_SPI_DMA_TX_IRQHandler     DMA1_Stream4_IRQHandler
#define AHRS_SPI_DMA_RX_IRQHandler     DMA1_Stream3_IRQHandler 

#define SPI_BUFFER_SIZE 256

namespace Andromeda
{
    class AhrsSpiDriver : InterruptTemplate
    {
	private:
		DMA_Stream_TypeDef *_txStream;
		DMA_Stream_TypeDef *_rxStream;
		uint32_t _txTcFlag;
		uint32_t _rxTcFlag;

		char *_data;
		char _currentCommand;

        char _rxBuffer[SPI_BUFFER_SIZE];
        char _txBuffer[SPI_BUFFER_SIZE];
		
		short _txFull;  //the bytes which are filled in the tx buffer
		short _rxFull;  //the bytes that are full in the rx buffer
		short _lastTx;

		void configureSpi();
		void configureDma();
		void configureInterrupts();

        virtual void onInterruptDma1Stream4();
        virtual void onInterruptDma1Stream3();
        virtual void onInterruptSpi2();

		bool beginWritePacket(char *data, short length);
        bool isTransmitting();

	public:
		AhrsSpiDriver();
        void initialise();
		void poll();
    };
}
