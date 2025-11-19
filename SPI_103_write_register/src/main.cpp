# include <libopencm3/stm32/rcc.h> //rcc.h - reset and clock control
# include <libopencm3/stm32/gpio.h> //inputs outputs
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#define spi1_nss GPIO4

#define SETUP_RETR  0x04
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ARC         0
#define ARD         4


void RF24_write_register(uint8_t reg, uint8_t value);

void clock_setup(void){
	// rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE12_72MHZ]);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);

	rcc_periph_clock_enable(RCC_SPI1);
}

void gpio_setup(void) {
	//светодиод для отладки
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, spi1_nss ); //NSS
	//для SPI 
     gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,	
            																		GPIO5 | //CLK
                                            										GPIO7 );//MOSI
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,GPIO6);		//MISO																			
}

void spi1_setup(void){
	spi_disable(SPI1);

	 spi_init_master(SPI1, 
					SPI_CR1_BAUDRATE_FPCLK_DIV_64,   	// Предделитель: делит частоту мк 
					SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, 	// Полярность: низкий уровень в idle
                    SPI_CR1_CPHA_CLK_TRANSITION_1,   	// Фаза: данные захватываются по левому фронту
				    SPI_CR1_DFF_8BIT,					// 8-битный формат данных
					SPI_CR1_MSBFIRST);					// тарший бит первый

	 // Дополнительные настройки
    spi_set_full_duplex_mode(SPI1);                  // Полнодуплексный режим
    spi_enable_software_slave_management(SPI1);      // Программное управление NSS
    spi_set_nss_high(SPI1);                          // NSS всегда высокий		
	
	spi_enable(SPI1);
}

int main() {
	clock_setup();
    gpio_setup();
	spi1_setup();
	
	
	while (true) {
		gpio_toggle(GPIOB,GPIO2);
		// gpio_toggle(GPIOA,spi1_nss);
		for(volatile uint32_t i =0; i < 1'000'000; i +=2);
		RF24_write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));
		
		
	}
}

void RF24_write_register(uint8_t reg, uint8_t value)
{
//   csn(LOW);
//   status = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
//   SPI.transfer(value);
//   csn(HIGH);

  gpio_clear(GPIOA,spi1_nss);
 
  spi_send(SPI1, (W_REGISTER | ( REGISTER_MASK & reg )) );
  // ждем, пока отправятся данные	(пока бит TXE регистра SPI_SR не установлен)
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};
    
  spi_send(SPI1,value);
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)){__asm__("nop");};

  // ждем, пока освободится шина
  while (SPI_SR(SPI1) & SPI_SR_BSY);
  
 
  gpio_set(GPIOA,spi1_nss);

  //сделать задержку // ждем реакции датчика
	//delay_us(ADNS3080_T_SWW);

//   return status;
}