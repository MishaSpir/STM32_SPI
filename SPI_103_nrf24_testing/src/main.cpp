# include "../inc/setup.hpp"
#include "../inc/time_setup.hpp"
#include "../inc/NRF.hpp"
// # include <libopencm3/stm32/timer.h>
// # include <libopencm3/cm3/nvic.h>


uint8_t counter = 9;




#define CE_PIN   GPIO0 
#define CE_PORT  GPIOB 
 
#define NSS_PIN  GPIO4 //это ss для SPI
#define NSS_PORT GPIOA 

uint16_t reg_value;


int main() {
	clock_setup();
    gpio_setup();
	gpio_set_mode(NSS_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, NSS_PIN ); // SPI NSS
    gpio_set_mode(CE_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, CE_PIN );   // CHIP ENABLE
	spi1_setup();
	timer2_setup();
	timer3_setup();
	uart2_setup();
	 delay_ms(1);

	
	
	RF24 radio(CE_PIN,CE_PORT,NSS_PIN,NSS_PORT);
	radio.ce(0);
    radio.csn(1);
	radio.begin();  
	// radio.write_register( RF_SETUP, 0x00 ) ;
	// radio.setPALevel( RF24_PA_MAX ) ;
	// radio.read_register( RF_SETUP) ;


	delay_ms(1000);
	// uint8_t setup = (uint8_t)radio.read_register(RF_SETUP) ;         // читат текущее состояние регистра RF_SETUP

	// // radio.write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));
	// //  uint8_t setup =(uint8_t)radio.read_register(SETUP_RETR);
 	// usart_send_blocking(USART2,setup);
	
// 	setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;  // очищает биты мощности



//     uint8_t level = RF24_PA_MAX; 
//   // switch uses RAM (evil!)                        // пишет новую мощность
//   if ( level == RF24_PA_MAX )
//   {
//     setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
//   }
//   else if ( level == RF24_PA_HIGH )
//   {
//     setup |= _BV(RF_PWR_HIGH) ;
//   }
//   else if ( level == RF24_PA_LOW )
//   {
//     setup |= _BV(RF_PWR_LOW);
//   }
//   else if ( level == RF24_PA_MIN )
//   {
//     // nothing
//   }
//   else if ( level == RF24_PA_ERROR )
//   {
//     // On error, go to maximum PA
//     setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
//   }

//   radio.write_register( RF_SETUP, setup ) ;




//   delay_ms(1);
//   radio.setAutoAck(1);       //0x21 0x3F
//   delay_ms(1);
//   radio.setRetries(0, 15);   //0x24 0x0F
//   delay_ms(1);
//   radio.enableAckPayload();  //0x1D_0xFF  0x3D_0x06 0x1D_0xFF 0x1C_0xFF 0x3C_0x03  
//   delay_ms(1);
//   radio.setPayloadSize(32);   // размер пакета, в байтах
//   delay_ms(1);
//   // radio.openWritingPipe(address[0]);  // мы - труба 0, открываем канал для передачи данных
//   // radio.openWritingPipe(0x314E6F6465); // "1Node" в hex
//   radio.openReadingPipe(1,0x314E6F6465); // "1Node" в hex
//   delay_ms(1);
//   radio.setChannel(0x60);             // выбираем канал (в котором нет шумов!)
//   delay_ms(1);
//   radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
//   delay_ms(1);
//   radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
//   //должна быть одинакова на приёмнике и передатчике!
//   delay_ms(1);
//   radio.powerUp();        // начать работу
//   delay_ms(1);
//   radio.startListening(); // начинаем слушать эфир, мы приёмный модуль

// 	delay_ms(5000);
		
 	

		// radio.setChannel(76);
	
	// reg_value = radio.read_register(RF_CH);

	while (true) {
		// uint8_t pipeNo, gotByte;
  		// while (radio.available(&pipeNo)) {        // слушаем эфир со всех труб
    	// 	radio.read(&gotByte, sizeof(gotByte));  // чиатем входящий сигнал
    	// 	// Serial.print("Recieved: ");
    	// 	// Serial.println(gotByte);
		// } 
		// delay_ms(10);
		// radio.write(&counter, sizeof(counter));
  		// counter++;

		delay_ms(100);
		radio.setPALevel( RF24_PA_MAX ) ;
		uint8_t setup = (uint8_t)radio.read_register(RF_SETUP) ;         // читат текущее состояние регистра RF_SETUP
 	    usart_send_blocking(USART2,setup);
	}
}

