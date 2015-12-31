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
    float m_nSteering;
    float m_nSpeed;
    char timestamp;
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
    char timestamp;
    unsigned short int   Checksum;
    char dummy;
};

class serialdriver:InterruptTemplate{
public:
    char _rx_buff[4*sizeof(CommandPacket)];
    char _rx_buff_cp[4*sizeof(CommandPacket)];
    char _out_buff[sizeof(Transmit_CommandPacket)];
    void RCC_Configuration(void);
    void GPIO_Configuration(void);
    void USART3_Configuration(void);
    void DMA_Configuration( char *_tx_data, char *_rx_data, int tx_size, int rx_size);
    void DMA1_Stream3_IRQHandler(void);
    void DMA1_Stream1_IRQHandler(void);
    void NVIC_Configuration(void);
    bool ChkChksum(CommandPacket* _data);
    void AddChecksum(Transmit_CommandPacket &_data);
    void Sendpack(Transmit_CommandPacket& _data);
    bool ReadPacket(CommandPacket* _data);
    int  findPackageHeader(unsigned char *buffer, short length);
    char timestamp_time;
    void FlushBuffers( void );
};
//#endif
