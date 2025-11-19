#ifndef LIQUIDCRYSTALSTM_HPP
#define LIQUIDCRYSTALSTM_HPP
#include <inttypes.h> //для uint8_t
#include <cstdlib> //для size_t
#include <stdint.h>
# include <libopencm3/stm32/rcc.h> 
# include <libopencm3/stm32/gpio.h> 
# include <libopencm3/stm32/timer.h>
# include <libopencm3/cm3/nvic.h>
#include <stdint.h>

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// =======================LiquidCrystalSTM DECLARE BEGIN=========================
class LiquidCrystal {
public:
//---КОНСТРУКТОРЫ-----------------------------------------------------------
	LiquidCrystal(uint16_t rs, uint16_t rw, uint16_t enable,
		uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3,
		uint16_t d4, uint16_t d5, uint16_t d6, uint16_t d7);

 	LiquidCrystal(uint16_t rs, uint16_t enable,
		uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3,
		uint16_t d4, uint16_t d5, uint16_t d6, uint16_t d7);

	LiquidCrystal(uint16_t rs, uint16_t rw, uint16_t enable,
		uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3);

 	LiquidCrystal(uint16_t rs, uint16_t enable,
		uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3);

//--ИНИЦИАЛИЗАЦИЯ-------------------------------------------------------------
	void init(uint8_t fourbitmode, uint16_t rs, uint16_t rw, uint16_t enable,
	    uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3,
	    uint16_t d4, uint16_t d5, uint16_t d6, uint16_t d7);	

	void begin(uint8_t cols, uint8_t lines, uint8_t dotsize);
	void setRowOffsets(uint8_t row0, uint8_t row1, uint8_t row2, uint8_t row3);	
	void command(uint8_t);
	void clear();
	void display();
	virtual size_t write(uint8_t);
  void home();
  void setCursor(uint8_t, uint8_t); 
  void createChar(uint8_t, uint8_t[]);
	
  
private:
	void send(uint8_t, uint8_t);
 	void write4bits(uint8_t);
	void pulseEnable();
	void write8bits(uint8_t);


//--ПЕРЕМЕННЫЕ--------------------------------------------------  
  uint16_t _rs_pin; // LOW: command. HIGH: character.
  uint16_t _rw_pin; // LOW: write to LCD. HIGH: read from LCD.
  uint16_t _enable_pin; // activated by a HIGH pulse.
  uint16_t _data_pins[8];

  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;

  uint8_t _initialized;

  uint8_t _numlines;
  uint8_t _row_offsets[4];

	
	};


// =======================LiquidCrystalSTM DECLARE END===========================


void pinWrite(uint32_t port,uint16_t pin, uint8_t val);
void lcd_timer2_setup(void); 
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);




 





#endif