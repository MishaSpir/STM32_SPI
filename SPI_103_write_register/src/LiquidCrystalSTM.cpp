#include "LiquidCrystalSTM.hpp"



// ========================TIMER2==================

volatile uint32_t tiks_us ;

void pinWrite(uint32_t port,uint16_t pin, uint8_t val){
	if (val == 0) {
        gpio_clear(port,pin);  // Установить бит в 0 (побитовое И с инверсией)
    } else {
         gpio_set(port,pin);   // Установить бит в 1 (побитовое ИЛИ)
    }
}



 void lcd_timer2_setup(void) {
    rcc_periph_clock_enable(RCC_TIM2);
	// 2. Настраиваем таймер:
	//Настройка PSC. Частота таймера = 72 МГц / ((36-1) + 1) = 2 MГц = 0.5 us
	//Максимальное число, кот можно записать в prescaler = 65535
	timer_set_prescaler(TIM2, 36-1);
	//настройка ARR Период = ((8-1) + 1) * 0.5 us = 4.0 us
	timer_set_period(TIM2,8-1);
	// 3. Настраиваем прерывание по обновлению
	timer_enable_irq(TIM2, TIM_DIER_UIE);
	// 4. Включаем таймер
	timer_enable_counter(TIM2);
	nvic_enable_irq(NVIC_TIM2_IRQ);

}

void tim2_isr(void) {//обработчик прерывания
	// Проверяем флаг прерывания
	if (timer_get_flag(TIM2, TIM_SR_UIF)) {
		// Сбрасываем флаг
		timer_clear_flag(TIM2, TIM_SR_UIF);
		// Переключаем светодиод
		// gpio_toggle(GPIOB, GPIO15);
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
  

//===============================================
















// =======================LiquidCrystalSTM DEFINE BEGIN=========================

LiquidCrystal::LiquidCrystal(uint16_t rs, uint16_t rw, uint16_t enable, 
		uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3,
		uint16_t d4, uint16_t d5, uint16_t d6, uint16_t d7)
{
  init(0, rs, rw, enable, d0, d1, d2, d3, d4, d5, d6, d7);
}

LiquidCrystal::LiquidCrystal(uint16_t rs,  uint16_t enable,
		uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3,
		uint16_t d4, uint16_t d5, uint16_t d6, uint16_t d7)
{
  init(0, rs, 255, enable, d0, d1, d2, d3, d4, d5, d6, d7);
}

 LiquidCrystal::LiquidCrystal(uint16_t rs, uint16_t rw, uint16_t enable,
		uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3)
{
  init(1, rs, rw, enable, d0, d1, d2, d3, 0, 0, 0, 0);
}

LiquidCrystal::LiquidCrystal(uint16_t rs, uint16_t enable,
		uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3)
{
  init(1, rs, 255, enable, d0, d1, d2, d3, 0, 0, 0, 0);
}



void LiquidCrystal::init(uint8_t fourbitmode, uint16_t rs, uint16_t rw, uint16_t enable,
	    uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3,
	    uint16_t d4, uint16_t d5, uint16_t d6, uint16_t d7)
{
  _rs_pin = rs;
  _rw_pin = rw;
  _enable_pin = enable;
  
  _data_pins[0] = d0;
  _data_pins[1] = d1;
  _data_pins[2] = d2;
  _data_pins[3] = d3; 
  _data_pins[4] = d4;
  _data_pins[5] = d5;
  _data_pins[6] = d6;
  _data_pins[7] = d7; 

  if (fourbitmode)
    _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
  else 
    _displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
  
  begin(16, 1,LCD_5x8DOTS);  
}

void LiquidCrystal:: begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (lines > 1) {
    _displayfunction |= LCD_2LINE;
  }
  _numlines = lines;

  setRowOffsets(0x00, 0x40, 0x00 + cols, 0x40 + cols);  

  // for some 1 line displays you can select a 10 pixel high font
  if ((dotsize != LCD_5x8DOTS) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;
  }

//   pinMode(_rs_pin, OUTPUT);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, _rs_pin);
  // we can save 1 pin by not using RW. Indicate by passing 255 instead of pin#
  if (_rw_pin != 255) { 
    // pinMode(_rw_pin, OUTPUT);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, _rw_pin);
	
  }
//   pinMode(_enable_pin, OUTPUT);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, _enable_pin);

  
  // Do these once, instead of every time a character is drawn for speed reasons.
  for (int i=0; i<((_displayfunction & LCD_8BITMODE) ? 8 : 4); ++i)
  {
    // pinMode(_data_pins[i], OUTPUT);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, _data_pins[i]);
	
   } 

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40 ms after power rises above 2.7 V
  // before sending commands. Arduino can turn on way before 4.5 V so we'll wait 50
  delay_us(50000); 
  // Now we pull both RS and R/W low to begin commands
//   digitalWrite(_rs_pin, LOW);
  pinWrite(GPIOB,_rs_pin, 0);
//   digitalWrite(_enable_pin, LOW);
   pinWrite(GPIOB,_enable_pin, 0);
  if (_rw_pin != 255) { 
// digitalWrite(_rw_pin, LOW);
	pinWrite(GPIOB,_rw_pin, 0);
  }
  
  //put the LCD into 4 bit or 8 bit mode
  if (! (_displayfunction & LCD_8BITMODE)) {
    // this is according to the Hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    write4bits(0x03);
    delay_us(4500); // wait min 4.1ms

    // second try
    write4bits(0x03);
    delay_us(4500); // wait min 4.1ms
    
    // third go!
    write4bits(0x03); 
    delay_us(150);

    // finally, set to 4-bit interface
    write4bits(0x02); 
  } else {
    // this is according to the Hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
    command(LCD_FUNCTIONSET | _displayfunction);
    delay_us(4500);  // wait more than 4.1 ms

    // second try
    command(LCD_FUNCTIONSET | _displayfunction);
    delay_us(150);

    // third go
    command(LCD_FUNCTIONSET | _displayfunction);
  }

  // finally, set # lines, font size, etc.
  command(LCD_FUNCTIONSET | _displayfunction);  

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // set the entry mode
  command(LCD_ENTRYMODESET | _displaymode);

}

void LiquidCrystal::setRowOffsets(uint8_t row0, uint8_t row1, uint8_t row2, uint8_t row3)
{
  _row_offsets[0] = row0;
  _row_offsets[1] = row1;
  _row_offsets[2] = row2;
  _row_offsets[3] = row3;
}

void LiquidCrystal::pulseEnable(void) {
  pinWrite(GPIOB,_enable_pin, 0);
  delay_us(4);    
  pinWrite(GPIOB,_enable_pin, 1);
  delay_us(4);    // enable pulse must be >450 ns
  pinWrite(GPIOB,_enable_pin, 0);
  delay_us(100);   // commands need >37 us to settle
}

void LiquidCrystal::write4bits(uint8_t value) {
  for (int i = 0; i < 4; i++) {
    pinWrite(GPIOB,_data_pins[i], (value >> i) & 0x01);
  }

  pulseEnable();
}

void LiquidCrystal::write8bits(uint8_t value) {
  for (int i = 0; i < 8; i++) {
    pinWrite(GPIOB,_data_pins[i], (value >> i) & 0x01);
  }
  
  pulseEnable();
}

void LiquidCrystal::send(uint8_t value, uint8_t mode) {
//   digitalWrite(_rs_pin, mode);
    pinWrite(GPIOB,_rs_pin, mode);


  // if there is a RW pin indicated, set it low to Write
  if (_rw_pin != 255) { 
    // digitalWrite(_rw_pin, LOW);
    pinWrite(GPIOB,_rw_pin, 0);

  }
  
  if (_displayfunction & LCD_8BITMODE) {
    write8bits(value); 
  } else {
    write4bits(value>>4);
    write4bits(value);
  }
}

inline void LiquidCrystal::command(uint8_t value) {
  send(value, 0);
}


void LiquidCrystal::clear()
{
  command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  delay_us(2000);  // this command takes a long time!
}

void LiquidCrystal::display() {
  _displaycontrol |= LCD_DISPLAYON;
  command(LCD_DISPLAYCONTROL | _displaycontrol);
}

inline size_t LiquidCrystal::write(uint8_t value) {
  send(value, 1);
  return 1; // assume success
}



void LiquidCrystal::home()
{
  command(LCD_RETURNHOME);  // set cursor position to zero
  delay_us(2000);  // this command takes a long time!
}

void LiquidCrystal::setCursor(uint8_t row, uint8_t col)
{
  const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
  if ( row >= max_lines ) {
    row = max_lines - 1;    // we count rows starting w/ 0
  }
  if ( row >= _numlines ) {
    row = _numlines - 1;    // we count rows starting w/ 0
  }
  
  command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}

void LiquidCrystal::createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    write(charmap[i]);
  }
}
// =======================LiquidCrystalSTM DEFINE END===========================