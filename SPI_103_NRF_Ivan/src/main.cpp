# include "../inc/setup.hpp"
#include "../inc/time_setup.hpp"
#include "../inc/NRF.hpp"
#include "../inc/nrf24l01.h"
#include "../inc/R24_config.h"
// # include <libopencm3/stm32/timer.h>
// # include <libopencm3/cm3/nvic.h>


uint8_t counter = 9;




#define CE_PIN   GPIO0 
#define CE_PORT  GPIOB 
 
#define NSS_PIN  GPIO4 //это ss для SPI
#define NSS_PORT GPIOA 

uint16_t reg_value;


int main() {
	uint64_t t = 0;
	clock_setup();
    gpio_setup();
	gpio_set_mode(NSS_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, NSS_PIN ); // SPI NSS
    gpio_set_mode(CE_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, CE_PIN );   // CHIP ENABLE
	spi1_setup();
	timer2_setup();
	timer3_setup();
	
    delay_ms(2000);
	
	
	NRF24 radio(CE_PIN,CE_PORT,NSS_PIN,NSS_PORT);
	 
	radio.begin();
	 delay_ms(1);
  	radio.setAutoAck(1);       //должен быть 1
  	 delay_ms(1);
 	radio.setRetries(3, 15);   //0x24 0x0F
 	 delay_ms(1);
  	radio.enableAckPayload();  //0x1D_0x00  0x3D_0x06 0x1D_0x00 0x1C_0xFF 0x3C_0x03  
	 delay_ms(1);
	radio.setPayloadSize(32);   // размер пакета, в байтах
	

	// radio.openWritingPipe(address[0]);  
	radio.openWritingPipe(0x314E6F6465); // "1Node" в hex
	 delay_ms(1);
    radio.setChannel(0x70);             // выбираем канал (в котором нет шумов!)

	delay_ms(1);
  	radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  	delay_ms(1);
  	radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  	//должна быть одинакова на приёмнике и передатчике!


	delay_ms(1);
  	radio.powerUp();        // начать работу
  	delay_ms(1);
  	radio.stopListening(); 

    //   radio.flush_rx();
  	//   radio.flush_tx();
	  delay_ms(5000);

	 

		// radio.setChannel(76);
	
	// reg_value = radio.read_register(RF_CH);

	while (true) {
		// if((get_ms() - t) >= 100){
			t = get_ms();
		gpio_set(GPIOB,GPIO2);
		delay_ms(1);	
		gpio_clear(GPIOB,GPIO2);
		// radio.powerUp();  
		radio.write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
		delay_ms(10);
		radio.write(&counter, sizeof(counter));
  		
		counter++;


		//   bool tx_ok, tx_fail,ack_payload_available;
		// radio.whatHappened(tx_ok,tx_fail,ack_payload_available);

		delay_ms(1000);	
		// radio.write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
		// }	
	}
}


