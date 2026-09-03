#ifndef __NRF204L01_H
#define __NRF204L01_H

void NRF24L01_GPIO_Init(void);
uint8_t NRF24L01_R_IRQ(void);
void Write_NRF24L01_Val(uint8_t Reg,uint8_t Value);
uint8_t Read_NRF24L01_Val(uint8_t Reg);
void Write_NRF24L01_Buf(uint8_t Reg,uint8_t *Buf,uint8_t Len);
void Receive_NRF24L01_Buf(uint8_t Reg,uint8_t *Buf,uint8_t Len);
void NRF24L01_Init(void);
void Receive_NRF24L01_Data(uint8_t*Buf);
uint8_t Send(uint8_t *Buf);

#endif
