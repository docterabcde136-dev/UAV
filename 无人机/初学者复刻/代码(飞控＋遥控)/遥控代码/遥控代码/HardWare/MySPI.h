#ifndef __MYSPI_H
#define __MYSPI_H


void MySPI_W_MOSI(uint8_t Value);
void MySPI_W_SCLK(uint8_t Value);
void MySPI_W_CSN(uint8_t Value);
void MySPI_W_CE(uint8_t Value);
uint8_t MySPI_R_MISO(void);
uint8_t MySPI_SwapData(uint8_t Byte);



#endif
