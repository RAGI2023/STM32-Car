#include "scope.h"
#include "uart_dma.h" // 引用您项目自己的UART DMA头文件
#include <string.h>

/**
 * @brief  向上位机发送通道名称 (非阻塞方式)
 */
void Scope_SendNames(const char* names)
{
    const uint8_t header[] = "AABBCC";
    const uint8_t tail[]   = "CCBBAA";
    uint8_t header_len = strlen((char*)header);
    uint8_t tail_len = strlen((char*)tail);
    uint8_t names_len = strlen(names);
    
    // 创建一个足够大的临时缓冲区来组装完整的包
    uint8_t full_packet[128]; // 缓冲区设大一点以防万一
    
    // 1. 组装包: 帧头 + 名称 + 帧尾
    memcpy(full_packet, header, header_len);
    memcpy(full_packet + header_len, names, names_len);
    memcpy(full_packet + header_len + names_len, tail, tail_len);
    
    // 2. 使用项目自带的非阻塞发送函数，将完整的包放入发送队列
    //    串口号1对应调试口
    UartTxDataQueue(1, full_packet, header_len + names_len + tail_len);
}

/**
 * @brief  将浮点数据打包并通过非阻塞队列发送
 */
void Scope_SendData(float* data, uint8_t num_channels)
{
    const uint8_t header[] = "DDEEFF";
    const uint8_t tail[]   = "FFEEDD";
    uint8_t header_len = strlen((char*)header);
    uint8_t tail_len = strlen((char*)tail);
    uint8_t data_len = num_channels * sizeof(float);
    
    // 创建一个临时缓冲区来组装完整的包
    uint8_t full_packet[32]; // 6(头) + 2*4(数据) + 6(尾) = 20, 32足够
    
    // 1. 组装包: 帧头 + 数据 + 帧尾
    memcpy(full_packet, header, header_len);
    memcpy(full_packet + header_len, data, data_len);
    memcpy(full_packet + header_len + data_len, tail, tail_len);
    
    // 2. 使用项目自带的非阻塞发送函数，将完整的包放入发送队列
    //    这个函数不会阻塞PID控制环路。
    UartTxDataQueue(1, full_packet, header_len + data_len + tail_len);
}