#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "control_logic.h"

/**
 * serial_gravity_comp.c
 * 
 * 一个简单的 C 工具，用于为 Python 串口控制程序提供重力补偿计算服务。
 * 输入 (stdin): q1 q2 q3 q4 q5 q6 q7 qd1 qd2 qd3 qd4 qd5 qd6 qd7 (单位: rad, rad/s)
 * 输出 (stdout): tau1 tau2 tau3 tau4 tau5 tau6 tau7 (单位: N.m, 空格分隔)
 */

#include "main_stm.h"
#include <stdint.h>

/* 二进制协议定义 */
#pragma pack(push, 1)
typedef struct {
    uint16_t magic;      // 0xAA55
    uint8_t mode;        // 1: Control (Full stm_step), 2: FK only
    double q[7];
    double qd[7];
    double target_pos[3];
    double target_quat[4];
} bin_input_t;

typedef struct {
    uint16_t magic;      // 0xAA55
    int32_t status;
    double tau[7];
    double ee_pos[3];    // 传回当前计算出的末端位置 (用于 Python 可视化)
    double ee_quat[4];   // 传回当前计算出的末端姿态
    double calc_time_ms; // stm_step 内部真实计算耗时
} bin_output_t;
#pragma pack(pop)

int main() {
    // 0. 禁用标准输入输出缓冲，确保最低延迟
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    // 1. 初始化底层控制逻辑
    stm_init();

    bin_input_t bin_in;
    bin_output_t bin_out;
    stm_input_t stm_in;
    stm_output_t stm_out;

    bin_out.magic = 0xAA55;

    // 2. 循环读取二进制输入
    // 使用 fread 直接读取固定长度的结构体
    while (fread(&bin_in, sizeof(bin_input_t), 1, stdin) == 1) {
        if (bin_in.magic != 0xAA55) {
            fprintf(stderr, "[ERROR] Invalid binary magic: 0x%04X\n", bin_in.magic);
            continue;
        }

        if (bin_in.mode == 2) {
            /* 模式 2: 仅计算正运动学 (FK) */
            control_get_fk_with_offset(bin_in.q, bin_out.ee_pos, bin_out.ee_quat);
            bin_out.status = 0;
            bin_out.calc_time_ms = 0.0;
            memset(bin_out.tau, 0, sizeof(double) * 7);
        } else {
            /* 模式 1: 执行完整的 stm_step (控制 + 轨迹 + 安全) */
            memcpy(stm_in.q, bin_in.q, sizeof(double) * 7);
            memcpy(stm_in.qd, bin_in.qd, sizeof(double) * 7);
            memcpy(stm_in.target_pos, bin_in.target_pos, sizeof(double) * 3);
            memcpy(stm_in.target_quat, bin_in.target_quat, sizeof(double) * 4);

            stm_step(&stm_in, &stm_out);

            bin_out.status = stm_out.status;
            memcpy(bin_out.tau, stm_out.tau, sizeof(double) * 7);
            memcpy(bin_out.ee_pos, stm_out.ee_pos, sizeof(double) * 3);
            memcpy(bin_out.ee_quat, stm_out.ee_quat, sizeof(double) * 4);
            bin_out.calc_time_ms = stm_out.calc_time_ms;
            
            if (stm_out.status < 0) {
                fprintf(stderr, "\n[FATAL SAFETY ERROR] Stop triggered by main_stm.\n");
                // 在二进制模式下，我们也写回最后一帧并在 stderr 报错后退出
                fwrite(&bin_out, sizeof(bin_output_t), 1, stdout);
                exit(1);
            }
        }

        // 3. 写回二进制输出
        fwrite(&bin_out, sizeof(bin_output_t), 1, stdout);
        // 不需要 fflush，因为已设置 _IONBF
    }

    return 0;
}
