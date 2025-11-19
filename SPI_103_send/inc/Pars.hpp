#ifndef PARS_HPP
#define PARS_HPP
#include <libopencm3/stm32/usart.h>
#include <cstdint>
#include <cmath> 
#include  <cstdlib>

constexpr uint8_t SIZE{64};
class Circular_buffer {
public:
	void put(uint8_t);
	uint8_t get();
	uint8_t get_rd();
	uint8_t get_wr();
	bool empty();
	bool full();
	Circular_buffer(); // Circular_buffer b1;
	Circular_buffer(uint8_t); // Circular_buffer b2(128);
	uint8_t readBytes(uint8_t* str, uint8_t amount);
	uint8_t count;
	uint8_t buf[SIZE]; // Противоречит конструктору с пармаетрами
private:
	
	uint8_t wr_idx;
	uint8_t rd_idx;
	
	bool full_;
};

class Parsing{
	public:
	Parsing();
	void FSM();
	private:

};

void uart3_write(uint8_t* data, const uint32_t length );
void uart2_write(uint8_t* data, const uint32_t length );
void uart1_write(uint8_t* data, const uint32_t length );

char* reverse(char* buffer, int i, int j);

// сама функция itoa
char* itoa(int value, char* buffer, int base);

#endif