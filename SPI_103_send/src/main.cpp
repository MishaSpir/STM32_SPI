# include <libopencm3/stm32/rcc.h> //rcc.h - reset and clock control
# include <libopencm3/stm32/gpio.h> //inputs outputs
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>


void clock_setup(void){
	// rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE12_72MHZ]);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);

	rcc_periph_clock_enable(RCC_SPI1);
}

void gpio_setup(void) {
	//светодиод для отладки
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
	//для SPI 
     gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,	GPIO4 | //NSS
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
		for(volatile uint32_t i =0; i < 1'000'000; i +=2);
		spi_send(SPI1, 'H');
		spi_send(SPI1, 'e');
		spi_send(SPI1, 'l');
		spi_send(SPI1, 'l');
		spi_send(SPI1, 'o');
		

		
	}
}