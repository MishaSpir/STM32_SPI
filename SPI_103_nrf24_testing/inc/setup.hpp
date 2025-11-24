#ifndef SETUP_HPP
#define SETUP_HPP

# include <libopencm3/stm32/rcc.h> //rcc.h - reset and clock control
# include <libopencm3/stm32/gpio.h> //inputs outputs
# include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
# include <libopencm3/cm3/nvic.h>




void pinWrite(uint32_t port,uint16_t pin, uint8_t val);
void clock_setup(void);
void gpio_setup(void);
void spi1_setup(void);
void uart2_setup(void);


#endif
