#include <Arduino.h>
#include <SPI.h>

#define spi_ss 6   // пин CS, остальные на шине

#define SETUP_RETR  0x04
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ARC         0
#define ARD         4


void sendByte(uint8_t data);
uint8_t RF24_write_register(uint8_t reg, uint8_t value);