//#ifndef __FTDICOMSDRIVER_H
//#define __FTDICOMSDRIVER_H
#pragma once

#include "Utilities/Utils.h"
#include "Utilities/InterruptTemplate.h"

//-----------------------------------------------------------------------------
// Ftdi.h
//
// This file contains the implementation of the UART driver to communicate with
// the FTDI232 USB module
//
// Author: Nima Keivan
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// The UART port definitions used for the communciation
//-----------------------------------------------------------------------------
#define FTDI_USART                     USART3
#define FTDI_USART_CLK                 RCC_APB1Periph_USART3
#define FTDI_IRQn                      USART3_IRQn
#define FTDI_IRQHandler                USART3_IRQHandler

//-----------------------------------------------------------------------------
// The UART port DMA definitions
//-----------------------------------------------------------------------------
#define FTDI_DR_ADDRESS                0x40004804		//this is the address of the register that the DMA uses

#define FTDI_DMA                       DMA1
#define FTDI_DMAx_CLK                  RCC_AHB1Periph_DMA1

#define FTDI_TX_DMA_CHANNEL            DMA_Channel_4
#define FTDI_TX_DMA_STREAM             DMA1_Stream3
#define FTDI_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
#define FTDI_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
#define FTDI_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
#define FTDI_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
#define FTDI_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3

#define FTDI_RX_DMA_CHANNEL            DMA_Channel_4
#define FTDI_RX_DMA_STREAM             DMA1_Stream1
#define FTDI_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
#define FTDI_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
#define FTDI_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
#define FTDI_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
#define FTDI_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

#define FTDI_DMA_TX_IRQn               DMA1_Stream3_IRQn
#define FTDI_DMA_RX_IRQn               DMA1_Stream1_IRQn
#define FTDI_DMA_TX_IRQHandler         DMA1_Stream3_IRQHandler
#define FTDI_DMA_RX_IRQHandler         DMA1_Stream1_IRQHandler


#define FTDI_COMMAND_TIMEOUT            2		//command time in intervals of systick
#define FTDI_BAUD                       356000    //115200*3

#define FTDI_PACKET_DELIMITER1          0xD0 // 208
#define FTDI_PACKET_DELIMITER2          0xAA // 170

//-----------------------------------------------------------------------------
// Size of RX TX buffers
//-----------------------------------------------------------------------------
#define FTDI_UART_RX_BUFFER_SIZE				256
#define FTDI_UART_TX_BUFFER_SIZE				256
#define FTDI_INTERRUPT_SIZE					11
#define MAX_PACKET_SIZE						100

#pragma pack(1)
struct CommandPacket
{
    char m_cDelimiter1 = FTDI_PACKET_DELIMITER1;
    char m_cDelimiter2 = FTDI_PACKET_DELIMITER2;
    char m_cSize = sizeof(CommandPacket);
    int m_nSteering;
    int m_nSpeed;
    unsigned short int   Checksum;
};

#pragma pack(1)
struct Transmit_CommandPacket
{
    char m_cDelimiter1 = FTDI_PACKET_DELIMITER1;
    char m_cDelimiter2 = FTDI_PACKET_DELIMITER2;
    char m_cSize = sizeof(Transmit_CommandPacket);
    short int   Acc_x;
    short int   Acc_y;
    short int   Acc_z;
    short int   Gyro_x;
    short int   Gyro_y;
    short int   Gyro_z;
    short int   Mag_x;
    short int   Mag_y;
    short int   Mag_z;
    int   Enc_LB;
    int   Enc_LF;
    int   Enc_RB;
    int   Enc_RF;
    short int   ADC_LB;
    short int   ADC_LF_yaw;
    short int   ADC_LF_rol;
    short int   ADC_RB;
    short int   ADC_RF_yaw;
    short int   ADC_RF_rol;
    unsigned short int   Checksum;
};

class FtdiComsDriver : InterruptTemplate
{
public:
    FtdiComsDriver();

public:
void Initialize(GPIO_Pin txPin, GPIO_Pin rxPin, GPIO_Pin ctsPin, GPIO_Pin rtsPin);
bool ReadPacket(unsigned char *packetBuffer, short &lengthOut);
bool beginWritePacket(char *data, short length);
void AddChecksum(Transmit_CommandPacket &_data);
bool ChkChksum(CommandPacket* _data);
void USART3_IRQ(void);

#define RXBUFFERSIZE 4
uint8_t RxBuffer[RXBUFFERSIZE];


private:
void apiPreparePacket(short length,char apiIdent, char frameId, char *packetData);
void apiCopyPacketData(const char *data1, const short length1, const char * data2, const short length2);
void apiSendPacket();
int findPackageHeader(unsigned char *buffer, short length);
int findPackageLength(unsigned char *buffer, short length);
void my_configureUart();
void configureUart();
void configureInterrupts();

virtual void onInterruptDma1Stream1();
virtual void onInterruptDma1Stream3();
void onEndTx();
void onEndRx();

unsigned char _rxBuffer0[FTDI_INTERRUPT_SIZE];
unsigned char _rxBuffer1[FTDI_INTERRUPT_SIZE];
int _rxRead;	//the amount of bytes already read from the buffers

unsigned char _rxBuffer[FTDI_UART_RX_BUFFER_SIZE];
unsigned char _txBuffer[FTDI_UART_TX_BUFFER_SIZE];
unsigned char _packetBuffer[FTDI_UART_TX_BUFFER_SIZE];

int _txFull;  //the bytes which are filled in the tx buffer
int _rxFull;  //the bytes that are full in the rx buffer
int _lastTx; //the last amount of bytes the went out through the tx;
int _totalPacketLength; //the total length of the packet so far

//registers that hold information about transferred bytes
int _txCount;


GPIO_Pin m_TxPin;
GPIO_Pin m_RxPin;
GPIO_Pin m_CtsPin;
GPIO_Pin m_RtsPin;

};

//#endif // FTDICOMSDRIVER_H
