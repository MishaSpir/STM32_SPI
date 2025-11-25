# include "../inc/setup.hpp"
#include "../inc/time_setup.hpp"
#include "../inc/NRF.hpp"
// # include <libopencm3/stm32/timer.h>
// # include <libopencm3/cm3/nvic.h>


uint8_t counter = 5;




#define CE_PIN   GPIO0 
#define CE_PORT  GPIOB 
 
#define NSS_PIN  GPIO4 //это ss для SPI
#define NSS_PORT GPIOA 

uint16_t reg_value;


int main() {
	clock_setup();
    gpio_setup();
	spi1_setup();
	timer2_setup();
	timer3_setup();
	uart2_setup();
	

	
	
	RF24 radio(CE_PIN,CE_PORT,NSS_PIN,NSS_PORT);
	radio.begin();
	 delay_ms(1);
  	radio.setAutoAck(1);       //0x21 0x3F
  	 delay_ms(1);
 	radio.setRetries(0, 15);   //0x24 0x0F
 	 delay_ms(1);
  	radio.enableAckPayload();  //0x1D_0x00  0x3D_0x06 0x1D_0x00 0x1C_0xFF 0x3C_0x03  
	 delay_ms(1);
	radio.setPayloadSize(32);   // размер пакета, в байтах
	

	// radio.openWritingPipe(address[0]);  
	radio.openReadingPipe(1,0x314E6F6465); // "1Node" в hex
	 delay_ms(1);
    radio.setChannel(0x60);             // выбираем канал (в котором нет шумов!)

	delay_ms(1);
  	radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  	delay_ms(1);
  	radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  	//должна быть одинакова на приёмнике и передатчике!


	delay_ms(1);
  	radio.powerUp();        // начать работу
  	delay_ms(1);
  	radio.startListening(); // начинаем слушать эфир, мы приёмный модуль

    // //   radio.flush_rx();
  	// //   radio.flush_tx();
	delay_ms(5000);
		
 	

		// radio.setChannel(76);
	
	// reg_value = radio.read_register(RF_CH);
    // uint8_t status_reg;
	while (true) {
        delay_us(50);
		uint8_t pipeNo, gotByte;
  		while (radio.available(&pipeNo)) {        // слушаем эфир со всех труб
    		radio.read(&gotByte, sizeof(gotByte));  // чиатем входящий сигнал
           
            radio.write_register(STATUS, (1<<RX_DR) | (1<< MAX_RT) | (1<<TX_DS));
            

            delay_us(10);
            // status_reg = radio.read_register(STATUS);
            
            gotByte+=1;
            // radio.write_register(STATUS, (1 << RX_DR));
            usart_send_blocking(USART2,gotByte);
            
		} 
		
	}
}