#include "../inc/NRF.hpp"
#include "../inc/time_setup.hpp"
#include <algorithm>  //для функции min



/****************************************************************************/
 RF24::RF24(uint16_t _cepin, uint32_t _ceport, uint16_t _cspin,uint32_t _csport){
    ce_pin = _cepin;
    ce_port = _ceport;
    csn_port = _csport;
    csn_pin = _cspin;
 }
/****************************************************************************/

void RF24::csn(uint8_t mode)
{
  pinWrite(csn_port,csn_pin,mode);
}
/****************************************************************************/

void RF24::ce(uint8_t level)
{
  pinWrite(ce_port,ce_pin,level);
}
/****************************************************************************/
//функцию begin использовать после инициализаци SPI!!!
void RF24::begin(void){
    gpio_set_mode(csn_port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, csn_pin ); // SPI NSS
    gpio_set_mode(ce_port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, ce_pin );   // CHIP ENABLE
    
    ce(0);
    csn(1);

    delay_ms(5);
    
    write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));  

    // Restore our default PA level
  setPALevel( RF24_PA_MAX) ;

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
void RF24::write_register(uint8_t reg, uint8_t value)
{
  csn(0);
  spi_send(SPI1, (W_REGISTER | ( REGISTER_MASK & reg )) );
  // ждем, пока отправятся данные	(пока бит TXE регистра SPI_SR не установлен)
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  spi_send(SPI1,value);
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  // ждем, пока освободится шина
  while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
  // delay_us(24);
  csn(1);

  //сделать задержку // ждем реакции датчика
	

//   return status;
}
/****************************************************************************/

uint16_t RF24::read_register(uint8_t reg) 
{
    uint16_t result;
    csn(0);

    spi_send(SPI1, R_REGISTER | ( REGISTER_MASK & reg ));
    while (!(SPI_SR(SPI1) & SPI_SR_RXNE));

    // отправляем любой бит для получения данных из указанного регистра 
    spi_send(SPI1, 0x00);
	  while (!(SPI_SR(SPI1) & SPI_SR_RXNE));

    result = spi_read(SPI1);
    	// ждем, пока освободится шина
	  while (SPI_SR(SPI1) & SPI_SR_BSY);
    csn(1);

    return result;
}
/****************************************************************************/

void RF24::setPALevel(rf24_pa_dbm_e level)
{
  uint8_t setup = (uint8_t)read_register(RF_SETUP) ;         // читат текущее состояние регистра RF_SETUP
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
  uint8_t setup = (uint8_t)read_register(RF_SETUP) ;

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
  uint8_t config = (uint8_t)read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;
  
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
  write_register(RF_CH,std::min(channel,max_channel));
}

/****************************************************************************/

void RF24::flush_rx(void)
{
  csn(0);
  spi_send(SPI1, FLUSH_RX );
  // ждем, пока отправятся данные	(пока бит TXE регистра SPI_SR не установлен)
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  // ждем, пока освободится шина
  while (SPI_SR(SPI1) & SPI_SR_BSY);
  csn(1);
}

/****************************************************************************/

void RF24::flush_tx(void)
{
  csn(0);
  spi_send(SPI1, FLUSH_TX );
  // ждем, пока отправятся данные	(пока бит TXE регистра SPI_SR не установлен)
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  // ждем, пока освободится шина
  while (SPI_SR(SPI1) & SPI_SR_BSY);
  csn(1);
}

/****************************************************************************/