#ifndef __NRF24L01_REG_H
#define __NRF24L01_REG_H

//引脚定义---联系MySPI中的定义
#define IRQ_Port   GPIOB
#define CE_Port    GPIOA
#define MOSI_Port  GPIOB
#define MISO_Port  GPIOB
#define SCK_Port   GPIOA
#define CSN_Port   GPIOA

#define IRQ_Pin   GPIO_Pin_13
#define MOSI_Pin  GPIO_Pin_15
#define MISO_Pin  GPIO_Pin_14
#define SCK_Pin   GPIO_Pin_8
#define CSN_Pin   GPIO_Pin_11
#define CE_Pin    GPIO_Pin_12

 
// 寄存器地址
#define CONFIG      0x00    // 配置寄存器
#define EN_AA       0x01    // 自动应答使能
#define EN_RXADDR   0x02    // 接收地址使能
#define SETUP_AW    0x03    // 地址宽度设置
#define SETUP_RETR  0x04    // 自动重发设置
#define RF_CH       0x05    // 射频通道
#define RF_SETUP    0x06    // 射频设置
#define STATUS      0x07    // 状态寄存器
#define OBSERVE_TX  0x08    // 发送观测
#define CD          0x09    // 载波检测
#define RX_ADDR_P0  0x0A    // 数据管道0接收地址
#define RX_ADDR_P1  0x0B    // 数据管道1接收地址
#define RX_ADDR_P2  0x0C    // 数据管道2接收地址
#define RX_ADDR_P3  0x0D    // 数据管道3接收地址
#define RX_ADDR_P4  0x0E    // 数据管道4接收地址
#define RX_ADDR_P5  0x0F    // 数据管道5接收地址
#define TX_ADDR     0x10    // 发送地址
#define RX_PW_P0    0x11    // 数据管道0有效数据宽度
#define RX_PW_P1    0x12    // 数据管道1有效数据宽度
#define RX_PW_P2    0x13    // 数据管道2有效数据宽度
#define RX_PW_P3    0x14    // 数据管道3有效数据宽度
#define RX_PW_P4    0x15    // 数据管道4有效数据宽度
#define RX_PW_P5    0x16    // 数据管道5有效数据宽度
#define FIFO_STATUS 0x17    // FIFO状态寄存器
#define DYNPD       0x1C    // 动态有效数据宽度
#define FEATURE     0x1D    // 特征寄存器

// 命令
#define R_REGISTER    0x00  // 读寄存器
#define W_REGISTER    0x20  // 写寄存器
#define REGISTER_MASK 0x1F  // 寄存器掩码
#define ACTIVATE      0x50  // 激活命令
#define R_RX_PL_WID   0x60  // 读取接收数据宽度
#define R_RX_PAYLOAD  0x61  // 读取接收数据
#define W_TX_PAYLOAD  0xA0  // 写发送数据
#define W_ACK_PAYLOAD 0xA8  // 写应答数据
#define FLUSH_TX      0xE1  // 清空发送FIFO
#define FLUSH_RX      0xE2  // 清空接收FIFO
#define REUSE_TX_PL   0xE3  // 重用发送数据
#define NOP           0xFF  // 无操作(用于读取状态)

// CONFIG寄存器位定义
#define MASK_RX_DR    6     // 中断掩码: 接收数据就绪
#define MASK_TX_DS    5     // 中断掩码: 发送数据成功
#define MASK_MAX_RT   4     // 中断掩码: 最大重发次数
#define EN_CRC        3     // 启用CRC校验
#define CRCO          2     // CRC校验长度: 0=1字节, 1=2字节
#define PWR_UP        1     // 上电
#define PRIM_RX       0     // 模式选择: 0=发送, 1=接收


//状态
#define RX_OK   0x40
#define TX_OK   0x20
#define MAX_TX  0x10


// STATUS寄存器位定义
#define RX_DR         6     // 接收数据就绪
#define TX_DS         5     // 发送数据成功
#define MAX_RT        4     // 最大重发次数
#define RX_P_NO       1     // 接收数据管道号(位1-3)
#define TX_FULL       0     // 发送FIFO满

// RF_SETUP寄存器位定义
#define PLL_LOCK      4     // PLL锁定
#define RF_DR_LOW     5     // 低数据率
#define RF_DR_HIGH    3     // 高数据率
#define RF_PWR        1     // 射频功率(位1-2)

// FIFO_STATUS寄存器位定义
#define TX_REUSE      6     // 发送数据重用
#define TX_FIFO_FULL  5     // 发送FIFO满
#define TX_EMPTY      4     // 发送FIFO空
#define RX_FULL       1     // 接收FIFO满
#define RX_EMPTY      0     // 接收FIFO空

#endif
