#ifndef TIME_SETUP_HPP
#define TIME_SETUP_HPP
#include "../inc/setup.hpp"
# include <libopencm3/stm32/timer.h>



void timer2_setup(void);
void timer3_setup(void);
void tim2_isr(void);
void tim3_isr(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
uint32_t get_ms();
uint32_t get_us();

#endif