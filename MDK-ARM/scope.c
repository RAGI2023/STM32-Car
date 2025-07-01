#include "scope.h"
#include "uart_dma.h" 
#include <string.h>
#include <stdio.h>

/**
 * @brief  将4个浮点数打包成新协议格式，并通过非阻塞队列发送
 * @param  ch1-ch4: 4个通道的浮点数据
 * @retval None
 */
void Scope_Send4Floats(float ch1, float ch2, float ch3, float ch4)
{
    // 定义协议帧头 (5字节)
    const uint8_t header[] = {0x41, 0x42, 0x43, 0x44, 0x45};
    
    // 准备数据负载 (4个浮点数 = 16字节)
    float data_payload[4];
    data_payload[0] = ch1;
    data_payload[1] = ch2;
    data_payload[2] = ch3;
    data_payload[3] = ch4;

    // 创建一个缓冲区来组装完整的包 (5字节头 + 16字节数据 = 21字节)
    uint8_t full_packet[21];
    
    // 1. 组装包: 帧头 + 数据
    memcpy(full_packet, header, sizeof(header));
    memcpy(full_packet + sizeof(header), data_payload, sizeof(data_payload));
    
    // 2. 使用项目自带的非阻塞发送函数，将完整的包放入发送队列
    //    串口号1对应调试口
    UartTxDataQueue(1, full_packet, sizeof(full_packet));
}