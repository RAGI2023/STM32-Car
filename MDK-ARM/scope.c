#include "scope.h"
#include "uart_dma.h" 
#include <string.h>
#include <stdio.h>

/**
 * @brief  ��4���������������Э���ʽ����ͨ�����������з���
 * @param  ch1-ch4: 4��ͨ���ĸ�������
 * @retval None
 */
void Scope_Send4Floats(float ch1, float ch2, float ch3, float ch4)
{
    // ����Э��֡ͷ (5�ֽ�)
    const uint8_t header[] = {0x41, 0x42, 0x43, 0x44, 0x45};
    
    // ׼�����ݸ��� (4�������� = 16�ֽ�)
    float data_payload[4];
    data_payload[0] = ch1;
    data_payload[1] = ch2;
    data_payload[2] = ch3;
    data_payload[3] = ch4;

    // ����һ������������װ�����İ� (5�ֽ�ͷ + 16�ֽ����� = 21�ֽ�)
    uint8_t full_packet[21];
    
    // 1. ��װ��: ֡ͷ + ����
    memcpy(full_packet, header, sizeof(header));
    memcpy(full_packet + sizeof(header), data_payload, sizeof(data_payload));
    
    // 2. ʹ����Ŀ�Դ��ķ��������ͺ������������İ����뷢�Ͷ���
    //    ���ں�1��Ӧ���Կ�
    UartTxDataQueue(1, full_packet, sizeof(full_packet));
}