#include "../inc/setup.hpp"


void pinWrite(uint32_t port,uint16_t pin, uint8_t val){
	if (val == 0) {
        gpio_clear(port,pin);  // Установить бит в 0 (побитовое И с инверсией)
    } else {
         gpio_set(port,pin);   // Установить бит в 1 (побитовое ИЛИ)
    }
}


void clock_setup(void){
	rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_24MHZ]);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_USART2);
}

void gpio_setup(void) {
	//светодиод для отладки
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);

	//для отладки таймеров - выключи потом
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);

	
	//для SPI 
     gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,	
            																		GPIO5 | //CLK
                                            										GPIO7 );//MOSI
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,GPIO6);		//MISO																			
}

void spi1_setup(void){
	spi_disable(SPI1);

	 spi_init_master(SPI1, 
					SPI_CR1_BAUDRATE_FPCLK_DIV_256,   	// Предделитель: делит частоту мк 
					SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, 	// Полярность: низкий уровень в idle
                    SPI_CR1_CPHA_CLK_TRANSITION_1,   	// Фаза: данные захватываются по левому фронту
				    SPI_CR1_DFF_8BIT,					// 8-битный формат данных
					SPI_CR1_MSBFIRST);					// Старший бит первый

	 // Дополнительные настройки
    spi_set_full_duplex_mode(SPI1);                  // Полнодуплексный режим
    spi_enable_software_slave_management(SPI1);      // Программное управление NSS
    spi_set_nss_high(SPI1);                          // NSS всегда высокий		
	
	spi_enable(SPI1);
}

void uart2_setup(void){
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 9600);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	// //активируем перрывания по приёму данных в UART2
  	// USART_CR1(USART2) |= USART_CR1_RXNEIE; 
	// nvic_enable_irq(NVIC_USART2_IRQ);

	usart_enable(USART2);
}



 