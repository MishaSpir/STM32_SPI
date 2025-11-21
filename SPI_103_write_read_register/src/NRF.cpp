#include "../inc/NRF.hpp"


 RF24::RF24(uint16_t _cepin, uint32_t _ceport, uint16_t _cspin,uint32_t _csport){
    ce_pin = _cepin;
    ce_port = _ceport;
    csn_port = _csport;
    csn_pin = _cspin;
 }


void RF24::csn(uint8_t mode)
{
  pinWrite(csn_port,csn_pin,mode);
}


void RF24::ce(uint8_t level)
{
  pinWrite(ce_port,ce_pin,level);
}

void RF24::begin(void){
    gpio_set_mode(csn_port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, csn_pin ); // SPI NSS
    gpio_set_mode(ce_port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, ce_pin );   // CHIP ENABLE 
}


void RF24::write_register(uint8_t reg, uint8_t value)
{
  csn(0);
  spi_send(SPI1, (W_REGISTER | ( REGISTER_MASK & reg )) );
  // ждем, пока отправятся данные	(пока бит TXE регистра SPI_SR не установлен)
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  spi_send(SPI1,value);
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  // ждем, пока освободится шина
  while (SPI_SR(SPI1) & SPI_SR_BSY);
  csn(1);

  //сделать задержку // ждем реакции датчика
	//delay_us(ADNS3080_T_SWW);

//   return status;
}


uint16_t RF24::read_register(uint8_t reg) //функция в разработке
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
