#include "Pars.hpp"



void Circular_buffer::put(uint8_t d){
	buf[wr_idx] = d;
	wr_idx ++;
	wr_idx %= SIZE;
	if(wr_idx == rd_idx) full_ = true;

	count ++;
}

  

uint8_t Circular_buffer::get(){
	uint8_t d = buf[rd_idx];
	rd_idx++;
	rd_idx %= SIZE;
	return d;
}

uint8_t Circular_buffer::get_rd(void){
	return rd_idx;
}  

uint8_t Circular_buffer::get_wr(void){
	return wr_idx;
}  

bool Circular_buffer::empty(){return ((wr_idx == rd_idx) and (not full_));}
bool Circular_buffer::full(){return full_;}
Circular_buffer::Circular_buffer():wr_idx{},rd_idx{},full_{false}{}

uint8_t Circular_buffer::readBytes(uint8_t* str, uint8_t amount){
  if(count>=amount){
		for(int i=0; i<amount; ++i){
     uint8_t temp = get(); 
		 str[i]=buf[amount-wr_idx+i];//тут есть беда: если я начну записывать с конца буфера и перейду в начало, то...
		}
    wr_idx =0;
    rd_idx =0;
	  full_ = false; 
    count = 0;
    return 1;     
  }
  else{
    return 0;
  }
}






void uart3_write(uint8_t* data, const uint32_t length ){
	for(uint32_t i = 0; i < length; i++ ){
		usart_send_blocking(USART3,data[i]);
	}

}


void uart2_write(uint8_t* data, const uint32_t length ){
	for(uint32_t i = 0; i < length; i++ ){
		usart_send_blocking(USART2,data[i]);
	}

}

void uart1_write(uint8_t* data, const uint32_t length ){
	for(uint32_t i = 0; i < length; i++ ){
		usart_send_blocking(USART1,data[i]);
	}

}


char* reverse(char* buffer, int i, int j) {
  while (i < j) {
    char t = buffer[i];
    buffer[i++] = buffer[j];
    buffer[j--] = t;
  }

  return buffer;
}


char* itoa(int value, char* buffer, int base) {
  // мы поддерживаем только основания от 2 до 32
  if (base < 2 || base > 32) return NULL;

  // абсолютное значение числа
  int n = abs(value);
  int i = 0;

  // вычисляем цифры числа с конца, пока число не равно нулю
  do {
    // вычисляем очередную цифру в нужной степени
    int r = n % base;
    // записываем символ, который соответствует текущей цифре
    buffer[i++] = (char) ((r < 10) ? (r + '0') : (r - 10 + 'A'));
    // переходим к следующему числу
    n /= base;
  } while (n);

  // для отрицательных чисел в десятеричной системе
  // нужно добавить минус
  if (value < 0 && base == 10) buffer[i++] = '-';
  buffer[i] = '\0';

  // поскольку мы вычисляли цифры с конца, нужно перевернуть массив
  return reverse(buffer, 0, i - 1);
}

/*-----------------пример, как отправлять разные по значению, но одинаковые по длине int
int int_temp = 6; 
char char_temp[3];
itoa(int_temp,char_temp,10);

//ФОРМИРУЕМ ПАКЕТ ДАННЫХ (ПРОТОКОЛ)
	// usart_send_blocking(USART2,'$');
	// // usart_send_blocking(USART2,'1');
	// // usart_send_blocking(USART2,'2');
	// // usart_send_blocking(USART2,'3');
	// usart_send_blocking(USART2,char_temp[0]);
	// usart_send_blocking(USART2,char_temp[1]);
	// usart_send_blocking(USART2,char_temp[2]);
	// usart_send_blocking(USART2,',');
	// usart_send_blocking(USART2,'4');
	// usart_send_blocking(USART2,'6');
	// usart_send_blocking(USART2,'*');
*/	


	//ФОРМИРУЕМ ПАКЕТ ДАННЫХ (ПРОТОКОЛ)   ПРИМЕР
	// usart_send_blocking(USART2,'$');
	// usart_send_blocking(USART2,'1');
	// usart_send_blocking(USART2,'\0');
	// usart_send_blocking(USART2,'\0');
	// usart_send_blocking(USART2,'\0');
	// usart_send_blocking(USART2,',');
	// usart_send_blocking(USART2,'5');
	// usart_send_blocking(USART2,'6');
	// usart_send_blocking(USART2,'7');
	// usart_send_blocking(USART2,'\0');
	// usart_send_blocking(USART2,'*');