#include "../include/NRF.hpp"
#include <SPI.h>    

RF24 radio(9, 6); // "создать" модуль на пинах 9 и 6 

  bool p_var = 0;
 uint8_t reg_value;

void setup() {
  // запустить шину
  Serial.begin(9600);
  SPI.begin();
  radio.begin();    
  delay(2000);  
    // Flush buffers
  // radio.flush_rx();
  // radio.flush_tx();
  delay(100);  
  // radio.setPALevel( RF24_PA_HIGH );
  //  radio.setDataRate( RF24_2MBPS ) ;
  // radio.setCRCLength( RF24_CRC_16 );
  // radio.setChannel(76);
  delay(100);  
  // reg_value = radio.read_register(RF_CH);

//  reg_value = radio.read_register(SETUP_RETR);
  
}

void loop() {


  // radio.write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));
 
  // delay(100);  
  // // reg_value = radio.read_register(SETUP_RETR);
  // reg_value = radio.read_register(RF_SETUP);
  // Serial.println(reg_value,HEX);
  Serial.println(p_var);
  delay(1000);

}








