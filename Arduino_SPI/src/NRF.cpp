#include "../include/NRF.hpp"


RF24::RF24(uint8_t _cepin, uint8_t _cspin){
  ce_pin = _cepin;

  csn_pin = _cspin;
}

void RF24::csn(int mode)
{
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbs data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
#ifdef ARDUINO
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
#endif
  digitalWrite(csn_pin,mode);
}

void RF24::ce(int level)
{
  digitalWrite(ce_pin,level);
}


void RF24::begin(void)
{
  // Initialize pins
  pinMode(ce_pin,OUTPUT);
  pinMode(csn_pin,OUTPUT);
}

uint8_t RF24::write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  // IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x,%02x)\r\n"),reg,value));

  csn(LOW);
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0)); // начать
  status = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  SPI.transfer(value);
  SPI.endTransaction();        
  csn(HIGH);

  return status;
}

