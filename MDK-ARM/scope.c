#include "scope.h"
#include "uart_dma.h" // ��������Ŀ�Լ���UART DMAͷ�ļ�
#include <string.h>

/**
 * @brief  ����λ������ͨ������ (��������ʽ)
 */
void Scope_SendNames(const char* names)
{
    const uint8_t header[] = "AABBCC";
    const uint8_t tail[]   = "CCBBAA";
    uint8_t header_len = strlen((char*)header);
    uint8_t tail_len = strlen((char*)tail);
    uint8_t names_len = strlen(names);
    
    // ����һ���㹻�����ʱ����������װ�����İ�
    uint8_t full_packet[128]; // ���������һ���Է���һ
    
    // 1. ��װ��: ֡ͷ + ���� + ֡β
    memcpy(full_packet, header, header_len);
    memcpy(full_packet + header_len, names, names_len);
    memcpy(full_packet + header_len + names_len, tail, tail_len);
    
    // 2. ʹ����Ŀ�Դ��ķ��������ͺ������������İ����뷢�Ͷ���
    //    ���ں�1��Ӧ���Կ�
    UartTxDataQueue(1, full_packet, header_len + names_len + tail_len);
}

/**
 * @brief  ���������ݴ����ͨ�����������з���
 */
void Scope_SendData(float* data, uint8_t num_channels)
{
    const uint8_t header[] = "DDEEFF";
    const uint8_t tail[]   = "FFEEDD";
    uint8_t header_len = strlen((char*)header);
    uint8_t tail_len = strlen((char*)tail);
    uint8_t data_len = num_channels * sizeof(float);
    
    // ����һ����ʱ����������װ�����İ�
    uint8_t full_packet[32]; // 6(ͷ) + 2*4(����) + 6(β) = 20, 32�㹻
    
    // 1. ��װ��: ֡ͷ + ���� + ֡β
    memcpy(full_packet, header, header_len);
    memcpy(full_packet + header_len, data, data_len);
    memcpy(full_packet + header_len + data_len, tail, tail_len);
    
    // 2. ʹ����Ŀ�Դ��ķ��������ͺ������������İ����뷢�Ͷ���
    //    ���������������PID���ƻ�·��
    UartTxDataQueue(1, full_packet, header_len + data_len + tail_len);
}