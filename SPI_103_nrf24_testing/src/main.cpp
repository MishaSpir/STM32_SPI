# include "../inc/setup.hpp"
#include "../inc/time_setup.hpp"
#include "../inc/NRF.hpp"
// # include <libopencm3/stm32/timer.h>
// # include <libopencm3/cm3/nvic.h>



// Это послание для меня в будущем - тебе надо настроить read_register на Arduino,
// Затем пошли команду чтения регистра  на модуль NRF
// Прочитай значение ригистра, выведи это значение в порт, посмотри сигналы
// ИЗУЧИ СИГНАЛЫ ВНИМАТЕЛЬНО

// проделай всё тоже самое на STM32


#define CE_PIN   GPIO0 
#define CE_PORT  GPIOB 
 
#define NSS_PIN  GPIO4 //это ss для SPI
#define NSS_PORT GPIOA 

uint16_t reg_value;


int main() {
	clock_setup();
    gpio_setup();
	spi1_setup();
	timer2_setup();
	timer3_setup();
	

	
	
	RF24 radio(CE_PIN,CE_PORT,NSS_PIN,NSS_PORT);
	radio.begin();
	
    //   radio.flush_rx();
  	//   radio.flush_tx();
	  delay_us(100);

		// radio.setChannel(76);
	
	// reg_value = radio.read_register(RF_CH);

	while (true) {
		gpio_toggle(GPIOB,GPIO2);
		    // radio.flush_rx();
			// delay_ms(5);
  	  		// radio.flush_tx();
		// gpio_toggle(GPIOB,GPIO5);
			// delay_ms(5);
		// gpio_toggle(GPIOA,spi1_nss);
		
		//  radio.write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));
		
		delay_ms(5);
		// reg_value = radio.read_register(SETUP_RETR);

		
	}
}


