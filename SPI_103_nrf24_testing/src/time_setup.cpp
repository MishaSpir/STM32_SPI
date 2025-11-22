#include "../inc/time_setup.hpp"

 uint32_t tiks_ms = 0;
 uint32_t tiks_us = 0;

//TIM2 отвечает за tiks_ms
//TIM3 отвечает за tiks_us 


void timer2_setup(void) {
    // 1. Включаем тактирование TIM2
	rcc_periph_clock_enable(RCC_TIM2);
	// 2. Настраиваем таймер:
	//Настройка PSC. Частота таймера = 8 МГц / ((8000-1) + 1) = 10 кГц
	timer_set_prescaler(TIM2,240-1);
	//настройка ARR Период = ((1000-1) + 1) / 10 кГц = 1 секунда
	timer_set_period(TIM2,100-1);
	// 3. Настраиваем прерывание по обновлению
	timer_enable_irq(TIM2, TIM_DIER_UIE);
	// 4. Включаем таймер
	timer_enable_counter(TIM2);
	nvic_enable_irq(NVIC_TIM2_IRQ);

}

void tim2_isr(void) {//обработчик прерывания
	// Проверяем флаг прерывания
    
	if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        //  gpio_toggle(GPIOB, GPIO5);
		// Сбрасываем флаг
		timer_clear_flag(TIM2, TIM_SR_UIF);
		// Переключаем светодиод
		tiks_ms++;
	}

}



void timer3_setup(void) {
    rcc_periph_clock_enable(RCC_TIM3);
	// 2. Настраиваем таймер:
	//Настройка PSC. Частота таймера = 72 МГц / ((36-1) + 1) = 2 MГц = 0.5 us
	//Максимальное число, кот можно записать в prescaler = 65535

	timer_set_prescaler(TIM3,12-1);			//для черной платы
	timer_set_period(TIM3,8-1);

	// timer_set_prescaler(TIM3,160-1);  //  для Дискавери
	// timer_set_period(TIM3,100-1);

	// 3. Настраиваем прерывание по обновлению
	timer_enable_irq(TIM3, TIM_DIER_UIE);
	// 4. Включаем таймер
	timer_enable_counter(TIM3);
	nvic_enable_irq(NVIC_TIM3_IRQ);

}

void tim3_isr(void) {//обработчик прерывания
	// Проверяем флаг прерывания
	if (timer_get_flag(TIM3, TIM_SR_UIF)) {
		// Сбрасываем флаг
        //  gpio_toggle(GPIOB, GPIO5);
		timer_clear_flag(TIM3, TIM_SR_UIF);
		// Переключаем светодиод
		// gpio_toggle(GPIOB, GPIO15);
		// tiks_us++;
        // gpio_toggle(GPIOA,GPIO1);
        // gpio_set(GPIOA,GPIO1);
        tiks_us++;

	}

}

//  Функция задержки в микросекундах
void delay_us(uint32_t us) {
	uint32_t start = tiks_us ;
	while (tiks_us - start < us /4 ) {
		 __asm__("nop");
		}
	}
 
//  Функция задержки в миллисекундах
void delay_ms(uint32_t ms) {
	while (ms--) {
		delay_us(1000); // 1000 мкс = 1 мс
	}
}

uint32_t get_ms(){
    return tiks_ms;
} 

uint32_t get_us(){
    return tiks_us;
} 