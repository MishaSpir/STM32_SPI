#include "../inc/NRF.hpp"
#include "../inc/time_setup.hpp"
#include <algorithm>  //для функции min



/****************************************************************************/
 RF24::RF24(uint16_t _cepin, uint32_t _ceport, uint16_t _cspin,uint32_t _csport):
  wide_band(true),
  p_variant(false),   
  payload_size(32),
  ack_payload_available(false),
  dynamic_payloads_enabled(false),
  pipe0_reading_address(0){
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
uint8_t RF24::write_register(uint8_t reg, uint8_t value)
{
   uint8_t status;

  csn(0);

  // ВАЖНО: Очищаем RX буфер перед началом операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }

  // Шаг 1: Отправляем команду записи
  spi_send(SPI1, (W_REGISTER | ( REGISTER_MASK & reg )) );
  // ждем, пока отправятся данные	(пока бит TXE регистра SPI_SR не установлен)
  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};

  // Шаг 2: Получаем статус регистра
  status = (uint8_t)spi_read(SPI1);

  // Шаг 3: Отправляем значение в регистр
  spi_send(SPI1,value);
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  // Очищаем буфер после операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1);
    }

  // ждем, пока освободится шина
  while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
  
  csn(1);
  return status;
}

/****************************************************************************/



uint8_t RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;
  
  csn(0);
    // ВАЖНО: Очищаем RX буфер перед началом операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }


  spi_send(SPI1, (W_REGISTER | ( REGISTER_MASK & reg )) );
  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};

  status = (uint8_t)spi_read(SPI1);
  while ( len-- ){
    // SPI.transfer(*buf++);
    spi_send(SPI1, *buf++ );
    while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};
  }

    // Очищаем буфер после операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1);
    }


  while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
  csn(1);

  return status;
}

/****************************************************************************/


uint8_t RF24::read_register(uint8_t reg) 
{
    uint8_t result;
    
    csn(0);
    
    // ВАЖНО: Очищаем RX буфер перед началом операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }
    
    // Шаг 1: Отправляем команду чтения
    spi_send(SPI1, R_REGISTER | (REGISTER_MASK & reg));
    while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
    (void)spi_read(SPI1); // Читаем и игнорируем статус
    
    // Шаг 2: Получаем данные регистра
    spi_send(SPI1, 0xFF);
    while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
    result = (uint8_t) spi_read(SPI1); // Это реальные данные!
    
    // Очищаем буфер после операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1);
    }
    
    while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
    csn(1);
    
    return result;
}




uint8_t RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len)// не уверен насчет read_spi в цикле
{
  uint8_t status;

  csn(0);
  // ВАЖНО: Очищаем RX буфер перед началом операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }

  spi_send(SPI1, R_REGISTER | ( REGISTER_MASK & reg ));
  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
  
    status   = (uint8_t) spi_read(SPI1);
    // ждем, пока освободится шина
	  // while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};

  while ( len-- ){
    // Очищаем перед каждым новым байтом
        while (SPI_SR(SPI1) & SPI_SR_RXNE) {
            (void)spi_read(SPI1);
        }

    spi_send(SPI1, 0xff);
	  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
    *buf++   = (uint8_t)spi_read(SPI1);
    // ждем, пока освободится шина
	  // while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
  }


  // Финальная очистка
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1);
    }

  while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
  csn(1);

  return status;
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

void RF24::setAutoAck(bool enable)
{
  if ( enable )
    write_register(EN_AA, 0b111111);
  else
    write_register(EN_AA, 0);
}

/****************************************************************************/


void RF24::setRetries(uint8_t delay, uint8_t count)
{
 write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

/****************************************************************************/

void RF24::toggle_features(void)
{
  csn(0);
  spi_send(SPI1, ACTIVATE );
  // ждем, пока отправятся данные	(пока бит TXE регистра SPI_SR не установлен)
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  spi_send(SPI1,0x73);
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  // ждем, пока освободится шина
  while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
  // delay_us(24);
  csn(1);
}

/****************************************************************************/

void RF24::enableAckPayload(void) 
{
  //
  // enable ack payload and dynamic payload features
  //

  write_register(FEATURE,(uint8_t)read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! read_register(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    write_register(FEATURE,(uint8_t)read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
  }

  // IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  write_register(DYNPD,(uint8_t)read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}

/****************************************************************************/

void RF24::setPayloadSize(uint8_t size)
{
  const uint8_t max_payload_size = 32;
  payload_size = std::min(size,max_payload_size);
}



/****************************************************************************/

void RF24::powerUp(void)
{
  write_register(CONFIG,(uint8_t)read_register(CONFIG) | _BV(PWR_UP));
}

/******************************************************************/
void RF24::powerDown(void)
{
  write_register(CONFIG,(uint8_t)read_register(CONFIG) & ~_BV(PWR_UP));
}
/****************************************************************************/

void RF24::stopListening(void)
{
  ce(0);
  flush_tx();
  flush_rx();
}

/****************************************************************************/


void RF24::startListening(void)
{
  write_register(CONFIG, (uint8_t)read_register(CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
  write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address)
    write_register(RX_ADDR_P0, reinterpret_cast<const uint8_t*>(&pipe0_reading_address), 5);

  // Flush buffers
  flush_rx();
  flush_tx();

  // Go!
  ce(1);

  // wait for the radio to come up (130us actually only needed)
  delay_us(130);
}


/****************************************************************************/

void RF24::startWrite( const void* buf, uint8_t len )
{
  // Transmitter power-up
  write_register(CONFIG, (uint8_t)( read_register(CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX) );
  delay_us(150);

  // Send the payload
  write_payload( buf, len );

  // Allons!
  ce(1);
  delay_us(16);
  ce(0);
}
 
/****************************************************************************/

uint8_t RF24::write_payload(const void* buf, uint8_t len)
{
  uint8_t status;

  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

  uint8_t data_len = std::min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Writing %u bytes %u blanks]",data_len,blank_len);
  
  csn(0);
 
  // ВАЖНО: Очищаем RX буфер перед началом операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }

  // Шаг 1: Отправляем команду записи
  spi_send(SPI1, W_TX_PAYLOAD );
  // ждем, пока отправятся данные	(пока бит TXE регистра SPI_SR не установлен)
  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};

  // Шаг 2: Получаем статус регистра
  status = (uint8_t)spi_read(SPI1);

  while ( data_len-- ){
    // SPI.transfer(*current++);
    spi_send(SPI1, *current++ );
    while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};
  }
  while ( blank_len-- ){
    // SPI.transfer(0);
    spi_send(SPI1,0x00 );
    while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};
  }
  
  
   // Очищаем буфер после операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1);
    }

  write_register(STATUS, (1<<RX_DR) | (1<< MAX_RT) | (1<<TX_DS));
  read_register(STATUS);

  while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
  // delay_us(24);
  csn(1);

  return status;
}

/****************************************************************************/

uint8_t RF24::read_payload(void* buf, uint8_t len)
{
  uint8_t status; 
  uint8_t* current = reinterpret_cast<uint8_t*>(buf);

  uint8_t data_len = std::min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  csn(0);
  // ВАЖНО: Очищаем RX буфер перед началом операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }

  // Отправляем команду чтения payload и читаем статус
  spi_send(SPI1, R_RX_PAYLOAD);

  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
  status   = (uint8_t) spi_read(SPI1);


  while ( data_len-- ){
     // ВАЖНО: // Очищаем RX буфер перед каждым новым байтом
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }

    // Читаем "пустые" байты если нужно (для фиксированного размера payload)
    spi_send(SPI1, 0xFF);
	  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
    *current++   = (uint8_t)spi_read(SPI1);
    
  }

  while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};

  while ( blank_len-- ){
    spi_send(SPI1, 0xff);
	  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
  }

  // ВАЖНО: Очищаем RX буфер в конце операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }


	while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
  csn(1);
  return status;
}



/****************************************************************************/

bool RF24::write( const void* buf, uint8_t len )
{
  bool result = false;

  // Begin the write
  startWrite(buf,len);

  // ------------
  // At this point we could return from a non-blocking write, and then call
  // the rest after an interrupt

  // Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
  // or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
  // is flaky and we get neither.

  // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
  // if I tighted up the retry logic.  (Default settings will be 1500us.
  // Monitor the send
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = get_ms();
  const uint32_t timeout = 500; //ms to wait for timeout
  do
  {
    status = read_register(OBSERVE_TX,&observe_tx,1);
    // IF_SERIAL_DEBUG(Serial.print(observe_tx,HEX));
  }
  while( ! ( status & ( _BV(TX_DS) | _BV(MAX_RT) ) ) && ( get_ms() - sent_at < timeout ) );

  // The part above is what you could recreate with your own interrupt handler,
  // and then call this when you got an interrupt
  // ------------

  // Call this when you get an interrupt
  // The status tells us three things
  // * The send was successful (TX_DS)
  // * The send failed, too many retries (MAX_RT)
  // * There is an ack packet waiting (RX_DR)
  bool tx_ok, tx_fail;
  whatHappened(tx_ok,tx_fail,ack_payload_available);
  
  //printf("%u%u%u\r\n",tx_ok,tx_fail,ack_payload_available);

  result = tx_ok;
  // IF_SERIAL_DEBUG(Serial.print(result?"...OK.":"...Failed"));

  // Handle the ack packet
  if ( ack_payload_available )
  {
    ack_payload_length = getDynamicPayloadSize();
    // IF_SERIAL_DEBUG(Serial.print("[AckPacket]/"));
    // IF_SERIAL_DEBUG(Serial.println(ack_payload_length,DEC));
  }

  // Yay, we are done.

  // Power down
  // powerDown();

  // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
  flush_tx();

  return result;
}
/****************************************************************************/

void RF24::whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  uint8_t status = write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Report to the user what happened
  tx_ok = status & _BV(TX_DS);
  tx_fail = status & _BV(MAX_RT);
  rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

uint8_t RF24::getDynamicPayloadSize(void)
{
  uint8_t result = 0;

  csn(0);
  // ВАЖНО: Очищаем RX буфер перед началом операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }
  
  spi_send(SPI1, R_RX_PL_WID);
  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
  (void)spi_read(SPI1); // Читаем и ИГНОРИРУЕМ статус!  


  spi_send(SPI1, 0xff);
  while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
  result   = (uint8_t) spi_read(SPI1);
    

   // Очищаем буфер и ждем завершения
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }
    
    
  // ждем, пока освободится шина
	while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};
  csn(1);

  return result;
}



/****************************************************************************/

bool RF24::available(void)
{
  return available(NULL);
}

/****************************************************************************/
bool RF24::available(uint8_t* pipe_num)
{
  uint8_t status = get_status();

  // Too noisy, enable if you really want lots o data!!
  //IF_SERIAL_DEBUG(print_status(status));

  bool result = ( status & _BV(RX_DR) );

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> RX_P_NO ) & 0b111;

    // Clear the status bit

    // ??? Should this REALLY be cleared now?  Or wait until we
    // actually READ the payload?

    write_register(STATUS,_BV(RX_DR) );
    // Также очищаем другие флаги если нужно
       

    // Handle ack payload receipt
    if ( status & _BV(TX_DS) )
    {
      write_register(STATUS,_BV(TX_DS));
    }
  }

  return result;
}


/****************************************************************************/

uint8_t RF24::get_status(void)
{
  uint8_t status;

  csn(0);
  // ВАЖНО: Очищаем RX буфер перед началом операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }
  
  spi_send(SPI1, NOP);
    while (!(SPI_SR(SPI1) & SPI_SR_RXNE)){__asm__("nop");};
    status   = (uint8_t) spi_read(SPI1);
    // ждем, пока освободится шина


  // ВАЖНО: Очищаем RX буфер после операции
    while (SPI_SR(SPI1) & SPI_SR_RXNE) {
        (void)spi_read(SPI1); // Читаем и игнорируем старые данные
    }
    
    
	  while (SPI_SR(SPI1) & SPI_SR_BSY){__asm__("nop");};

 
  csn(1);

  return status;
}

/****************************************************************************/



void RF24::openWritingPipe(uint64_t value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), 5);
  write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), 5);

  const uint8_t max_payload_size = 32;
  write_register(RX_PW_P0,std::min(payload_size,max_payload_size));
}

/****************************************************************************/



void RF24::openReadingPipe(uint8_t child, uint64_t address)
{
  // Если это труба 0, кэшируем адрес. Это нужно потому,
  // что openWritingPipe() перезапишет адрес трубы 0,
  // поэтому startListening() должен будет восстановить его.
  if (child == 0)
    pipe0_reading_address = address;

  if (child <= 5)  // Исправлено с 6 на 5 (трубы от 0 до 5)
  {
    // Локальные массивы вместо PROGMEM
    const uint8_t child_pipe[] = {
      RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
    };
    
    const uint8_t child_payload_size[] = {
      RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
    };
    
    const uint8_t child_pipe_enable[] = {
      ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
    };

    // Для труб 2-5, записываем только младший байт
    if (child < 2) {
      write_register(child_pipe[child], reinterpret_cast<const uint8_t*>(&address), 5);
    } else {
      write_register(child_pipe[child], reinterpret_cast<const uint8_t*>(&address), 1);
    }

    write_register(child_payload_size[child], payload_size);

    // Включаем трубу в регистре EN_RXADDR
    write_register(EN_RXADDR, (uint8_t)(EN_RXADDR) | _BV(child_pipe_enable[child]));
  }
}

/****************************************************************************/

bool RF24::read( void* buf, uint8_t len )
{
  // Fetch the payload
  read_payload( buf, len );

  write_register(STATUS, (1<<RX_DR) | (1<< MAX_RT) | (1<<TX_DS));

  // was this the last of the data available?
  return read_register(FIFO_STATUS) & _BV(RX_EMPTY);
}

/****************************************************************************/
void RF24::disableCRC( void )
{
  uint8_t disable = read_register(CONFIG) & ~_BV(EN_CRC) ;
  write_register( CONFIG, disable ) ;
}