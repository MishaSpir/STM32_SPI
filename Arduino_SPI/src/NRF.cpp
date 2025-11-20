#include "../include/NRF.h"

uint8_t RF24_write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  

  digitalWrite(spi_ss, LOW);
  status = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  SPI.transfer(value);
  digitalWrite(spi_ss, HIGH);

  return status;
}

void sendByte(uint8_t data) {
  digitalWrite(spi_ss, LOW);    // выбрать активным
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0)); // начать
  SPI.transfer(data);           // отправить
  SPI.endTransaction();         // закончить отправку
  digitalWrite(spi_ss, HIGH);   // деактивировать
}