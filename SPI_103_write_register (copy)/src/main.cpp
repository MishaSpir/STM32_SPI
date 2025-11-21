# include "../inc/setup.hpp"
#include "../inc/NRF.hpp"


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
	
	RF24 radio(CE_PIN,CE_PORT,NSS_PIN,NSS_PORT);
	radio.begin();

	while (true) {
		gpio_toggle(GPIOB,GPIO2);
		// gpio_toggle(GPIOA,spi1_nss);
		for(volatile uint32_t i =0; i < 1'000'000; i +=2);
		radio.write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));
		for(volatile uint32_t i =0; i < 1'000'00; i +=2);
		reg_value = radio.read_register(SETUP_RETR);

		
	}
}

