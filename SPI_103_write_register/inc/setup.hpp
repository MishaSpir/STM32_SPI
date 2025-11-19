#ifndef SETUP_HPP
#define SETUP_HPP


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h>


#define lcd_en GPIO15
#define lcd_rs GPIO14

#define lcd_d4 GPIO5
#define lcd_d5 GPIO6
#define lcd_d6 GPIO7
#define lcd_d7 GPIO8

  
#define BAUD_RATE (9600)
#define M0	GPIO0
#define M1	GPIO1

void clock_setup(void);
void gpio_setup(void);
void uart2_setup(void);
void uart1_setup(void);
void systick_setup(void);

#endif
