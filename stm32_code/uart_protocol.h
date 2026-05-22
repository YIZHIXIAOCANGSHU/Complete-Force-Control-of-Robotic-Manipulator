/**
 * @file uart_protocol.h
 * @brief Python上位机与STM32下位机之间的UART二进制通信协议头文件
 *
 * 此协议用于双空间阻抗控制数据下发与指令回传。
 * 为保证跨平台数据一致性，所有通信的浮点数使用IEEE-754双精度(double, 8 bytes)。
 * 结构体必须使用1字节对齐(#pragma pack(1))打包。
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 固定的帧头和帧尾标识，用于在数据流中寻址对齐
#define UART_FRAME_HEAD 0xAA55
#define UART_FRAME_TAIL 0x55AA

/* 命令字定义 */
#define CMD_CONTROL_STATE 0x01 // 下发状态与目标：上位机 -> 下位机
#define CMD_CONTROL_FORCE 0x02 // 回传力矩：下位机 -> 上位机

#pragma pack(push, 1) // 强制按1字节打包，取消默认的数据对齐和填充

/**
 * @struct UartControlStatePacket
 * @brief 下行数据包 (Python -> STM32): 下发控制状态与指令
 * 总大小: 2(head) + 1(cmd) + 168(payload) + 2(crc) + 2(tail) = 175 Bytes
 */
typedef struct {
  uint16_t head; ///< 帧头: 0xAA55
  uint8_t cmd;   ///< 命令字: 0x01

  // 载荷 (Payload)
  double target_pos[3];  ///< 笛卡尔空间目标末端位置 (x, y, z 单位m)
  double target_quat[4]; ///< 笛卡尔空间目标末端姿态 (w, x, y, z)
  double current_q[7];   ///< 当前1-7关节实际角度 (rad)
  double current_qd[7];  ///< 当前1-7关节实际速度 (rad/s)

  uint16_t crc16; ///< 校验码：对前面的 head, cmd，以及载荷做 ModBus CRC16 运算
  uint16_t tail;  ///< 帧尾: 0x55AA
} UartControlStatePacket;

/**
 * @struct UartControlForcePacket
 * @brief 上行数据包 (STM32 -> Python): 回传控制力矩
 * 总大小: 2(head) + 1(cmd) + 56(payload) + 2(crc) + 2(tail) = 63 Bytes
 */
typedef struct {
  uint16_t head; ///< 帧头: 0xAA55
  uint8_t cmd;   ///< 命令字: 0x02

  // 载荷 (Payload)
  double tau_out[7]; ///< 计算得出的1-7关节前馈控制力矩 (N*m)

  uint16_t crc16; ///< 校验码：对前面的 head, cmd, tau_out 做运算
  uint16_t tail;  ///< 帧尾: 0x55AA
} UartControlForcePacket;

#pragma pack(pop) // 恢复默认内存对齐方式

/**
 * @brief 计算标准的 Modbus CRC16
 * @param data 待计算的数据指针
 * @param length 数据长度(字节)
 * @return 16位CRC校验码
 */
uint16_t calculate_crc16(const uint8_t *data, uint32_t length);

/**
 * @brief 尝试从缓冲区中解析出下行 ControlState 结构
 * @param buffer 接收到的字节缓冲区流
 * @param buf_len 缓冲区当前拥有数据的长度
 * @param target_pos 传出：解析得到的目标位置
 * @param target_quat 传出：解析得到的目标姿态四元数
 * @param current_q 传出：解析得到的当前关节角度
 * @param current_qd 传出：解析得到的当前关节速度
 * @return true解析成功数据已被填入传出参数
 * false解析失败(长度不够，头尾错位或CRC校验不匹配)
 */
bool parse_control_state_packet(const uint8_t *buffer, uint32_t buf_len,
                                double target_pos[3], double target_quat[4],
                                double current_q[7], double current_qd[7]);

/**
 * @brief 将计算好的力矩打包成用于网络发送的字节流
 * @param tau_out 传入：1-7关节期望力矩
 * @param buffer 传出：将生成好的63字节数据包放置于此
 * @param max_len 外部缓冲区的最大长度，用于越界保护
 * @return 实际封装的字节数 (正常情况为63)，若缓冲区过小返回0
 */
uint32_t pack_control_force_packet(const double tau_out[7], uint8_t *buffer,
                                   uint32_t max_len);

#ifdef __cplusplus
}
#endif

#endif // UART_PROTOCOL_H
