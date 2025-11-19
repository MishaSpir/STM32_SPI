#include "setup.hpp"





 void clock_setup(void){
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	// rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_USART1);

}

 void gpio_setup(void){
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, M0); //для M0
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, M1); //для M1
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, GPIO2); //для светодиода	

	//дисплей
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, lcd_rs);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, GPIO2);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, lcd_en);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ	,GPIO_CNF_OUTPUT_PUSHPULL, lcd_d4);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, lcd_d5);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, lcd_d6);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, lcd_d7);
}

 void uart2_setup(void){
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, BAUD_RATE);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	//активируем перрывания по приёму данных в UART2
  	USART_CR1(USART2) |= USART_CR1_RXNEIE; 
	nvic_enable_irq(NVIC_USART2_IRQ);

	usart_enable(USART2);
}

 void uart1_setup(void){

/* Setup GPIO pin GPIO_USART1_TX. */
gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
gpio_set_mode(GPIOA, GPIO_MODE_INPUT,GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
/* Setup UART parameters. */
usart_set_baudrate(USART1, BAUD_RATE);
usart_set_databits(USART1, 8);
usart_set_stopbits(USART1, USART_STOPBITS_1);
usart_set_mode(USART1, USART_MODE_TX_RX);
usart_set_parity(USART1, USART_PARITY_NONE);
usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

//активируем перрывания по приёму данных в UART1
  	USART_CR1(USART1) |= USART_CR1_RXNEIE; 
	nvic_enable_irq(NVIC_USART1_IRQ);

usart_enable(USART1);

}

void systick_setup(void){
systick_set_frequency(1000,72'000'000);
systick_counter_enable();
systick_interrupt_enable();

}


  
