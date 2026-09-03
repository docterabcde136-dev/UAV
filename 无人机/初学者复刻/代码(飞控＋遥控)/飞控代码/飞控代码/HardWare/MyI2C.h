#ifndef __MYI2C_H
#define __MYI2C_H

void MyI2C_W_SCL(uint8_t BitValue);
void MyI2C_W_SDA(uint8_t BitValue);
uint8_t MyI2C_R_SDA(void);
void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_Receive(void);
void MyI2C_Send_ACK(uint8_t send_ack);
uint8_t MyI2C_Receive_ACK(void);

#endif
