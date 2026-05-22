/**
 * @file uart_protocol.c
 * @brief UART通信数据包拆装和CRC验证的具体实现
 */
#include "uart_protocol.h"
#include <string.h>

/**
 * @brief 标准 Modbus CRC16 校验计算
 *
 * Modbus CRC16 特性:
 * 1. 初始值为 0xFFFF
 * 2. 异或多项式为 0xA001
 * 3. 输入数据低字节在前，先处理
 */
uint16_t calculate_crc16(const uint8_t *data, uint32_t length) {
  uint16_t crc = 0xFFFF;
  for (uint32_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/**
 * @brief 解析并验证接收到的下行协议包 (175字节)
 * 此函数设计为可以放入 UART 中断或轮询队列中。每次收到一整包数据时调用。
 */
bool parse_control_state_packet(const uint8_t *buffer, uint32_t buf_len,
                                double target_pos[3], double target_quat[4],
                                double current_q[7], double current_qd[7]) {
  // 1. 检查长度是否满足一个完整帧
  if (buf_len < sizeof(UartControlStatePacket)) {
    return false;
  }

  // 把缓冲区内容按结构体解释
  UartControlStatePacket *pkt = (UartControlStatePacket *)buffer;

  // 2. 检测帧头、帧尾和命令字是否正确匹配
  if (pkt->head != UART_FRAME_HEAD || pkt->cmd != CMD_CONTROL_STATE ||
      pkt->tail != UART_FRAME_TAIL) {
    return false;
  }

  // 3. 计算CRC并校验 (对 head 到 current_qd 执行计算，共计171Bytes)
  // sizeof(UartControlStatePacket) - 4 是去掉了末尾的 crc16 (2B) 和 tail (2B)
  uint16_t crc = calculate_crc16(buffer, sizeof(UartControlStatePacket) - 4);
  if (crc != pkt->crc16) {
    return false; // 数据包损坏，丢弃本帧
  }

  // 4. 数据安全可靠，复制内存到全局/算法传入的数组中
  memcpy(target_pos, pkt->target_pos, sizeof(double) * 3);
  memcpy(target_quat, pkt->target_quat, sizeof(double) * 4);
  memcpy(current_q, pkt->current_q, sizeof(double) * 7);
  memcpy(current_qd, pkt->current_qd, sizeof(double) * 7);

  return true;
}

/**
 * @brief 组装即将发送的上行反溃力矩控制包 (63字节)
 * 收到上位机指令计算完毕后，调用此函数将7维力矩封装并附加校验
 */
uint32_t pack_control_force_packet(const double tau_out[7], uint8_t *buffer,
                                   uint32_t max_len) {
  // 1. 防御性检查缓冲区余量
  if (max_len < sizeof(UartControlForcePacket)) {
    return 0;
  }

  UartControlForcePacket pkt;

  // 2. 依次填充 帧头 / CMD / 数据
  pkt.head = UART_FRAME_HEAD;
  pkt.cmd = CMD_CONTROL_FORCE;
  memcpy(pkt.tau_out, tau_out, sizeof(double) * 7);

  // 3. 计算 CRC16 (对前 59 Bytes 运算)
  pkt.crc16 = calculate_crc16((const uint8_t *)&pkt,
                              sizeof(UartControlForcePacket) - 4);
  pkt.tail = UART_FRAME_TAIL;

  // 4. 拷贝到发出的发送池
  memcpy(buffer, &pkt, sizeof(UartControlForcePacket));

  return sizeof(UartControlForcePacket); // 成功返回 63
}
