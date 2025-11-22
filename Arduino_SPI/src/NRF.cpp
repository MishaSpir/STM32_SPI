#include "../include/NRF.hpp"

/****************************************************************************/

RF24::RF24(uint8_t _cepin, uint8_t _cspin){
  ce_pin = _cepin;

  csn_pin = _cspin;
}
/****************************************************************************/

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
/****************************************************************************/

void RF24::ce(int level)
{
  digitalWrite(ce_pin,level);
}

/****************************************************************************/

void RF24::begin(void)
{
  // Initialize pins
  pinMode(ce_pin,OUTPUT);
  pinMode(csn_pin,OUTPUT);

   // Initialize SPI bus
  SPI.begin();

  ce(LOW);
  csn(HIGH);

  delay( 5 ) ;

  write_register(SETUP_RETR,(B0100 << ARD) | (B1111 << ARC));  

  // Restore our default PA level
  setPALevel( RF24_PA_MAX ) ;

  if( setDataRate( RF24_250KBPS ) )
  {
    p_variant = true ;
  }

    // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  setDataRate( RF24_1MBPS ) ;

  // Initialize CRC and request 2-byte (16bit) CRC
  setCRCLength( RF24_CRC_16 ) ;
  
  // Disable dynamic payloads, to match dynamic_payloads_enabled setting
  write_register(DYNPD,0);

  // Reset current status
  // Notice reset and flush is the last thing we do
  write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  setChannel(76);

  // Flush buffers
  flush_rx();
  flush_tx();
}
/****************************************************************************/

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
/****************************************************************************/


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
/****************************************************************************/

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
/****************************************************************************/

bool RF24::setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      wide_band = false ;
    }
  }
  write_register(RF_SETUP,setup);

  // Verify our result
  if ( read_register(RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    wide_band = false;
  }

  return result;
}
/****************************************************************************/

void RF24::setCRCLength(rf24_crclength_e length)
{
  uint8_t config = read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;
  
  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  write_register( CONFIG, config ) ;
}
/****************************************************************************/
void RF24::setChannel(uint8_t channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

  const uint8_t max_channel = 127;
  write_register(RF_CH,min(channel,max_channel));
}

/****************************************************************************/

uint8_t RF24::flush_rx(void)
{
  uint8_t status;

  csn(LOW);
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0)); // начать
  status = SPI.transfer( FLUSH_RX );
  SPI.endTransaction(); 
  csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24::flush_tx(void)
{
  uint8_t status;

  csn(LOW);
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0)); // начать
  status = SPI.transfer( FLUSH_TX );
  SPI.endTransaction(); 
  csn(HIGH);

  return status;
}

/****************************************************************************/

