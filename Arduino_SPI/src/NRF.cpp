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

   // Initialize SPI bus
  SPI.begin();

  ce(LOW);
  csn(HIGH);

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


uint8_t RF24::read_register(uint8_t reg)
{
  csn(LOW);
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0)); // начать
  SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  uint8_t result = SPI.transfer(0xff);
  SPI.endTransaction();    
  csn(HIGH);
  return result;
}

void RF24::setPALevel(rf24_pa_dbm_e level)
{
  uint8_t setup = read_register(RF_SETUP) ;         // читат текущее состояние регистра RF_SETUP
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;  // очищает биты мощности

  // switch uses RAM (evil!)                        // пишет новую мощность
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  write_register( RF_SETUP, setup ) ;
}


