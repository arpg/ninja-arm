//#ifndef __USART3DMA_H
//#define __USART3DMA_H

#pragma once

#include "Utilities/InterruptTemplate.h"

#define USART3_BAUD                       76800    //fix it later: 115200 but had to adjust from oscillscope (115200*2/3)

#define USART3_PACKET_DELIMITER1          0xD0 // 208
#define USART3_PACKET_DELIMITER2          0xAA // 170

//////////////////////////////////////////////////////

#pragma pack(1)
struct CommandPacket
{
    char m_cDelimiter1 = USART3_PACKET_DELIMITER1;
    char m_cDelimiter2 = USART3_PACKET_DELIMITER2;
    char m_cSize = sizeof(CommandPacket);
    int m_nSteering;
    int m_nSpeed;
    unsigned short int   Checksum;
};

#pragma pack(1)
struct Transmit_CommandPacket
{
    char m_cDelimiter1 = USART3_PACKET_DELIMITER1;
    char m_cDelimiter2 = USART3_PACKET_DELIMITER2;
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
    short int   ADC_Steer;
    short int   ADC_LB;
    short int   ADC_LF;
    short int   ADC_RB;
    short int   ADC_RF;
    unsigned short int   Checksum;
};

class serialdriver:InterruptTemplate{
public:
    void RCC_Configuration(void);
    void GPIO_Configuration(void);
    void USART3_Configuration(void);
    void DMA_Configuration( char *_tx_data, char *_rx_data, int tx_size, int rx_size);
    void DMA1_Stream3_IRQHandler(void);
    void DMA1_Stream1_IRQHandler(void);
    void NVIC_Configuration(void);
//    bool ChkChksum(CommandPacket* _data);
//    void AddChecksum(Transmit_CommandPacket &_data);
//    bool ReadPacket(unsigned char *packetBuffer, short &lengthOut);
//    int  findPackageHeader(unsigned char *buffer, short length);
//    int  findPackageLength(unsigned char *buffer, short length);
//    void CopyToTransmitBuff( Transmit_CommandPacket* _data, short int size );
};
//#endif
