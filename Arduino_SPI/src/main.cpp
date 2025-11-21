#include "../include/NRF.hpp"
#include <SPI.h>    

RF24 radio(9, 6); // "создать" модуль на пинах 9 и 6 


 uint8_t reg_value;

void setup() {
  // запустить шину
  Serial.begin(9600);
  SPI.begin();
  radio.begin();    
  delay(2000);  
  reg_value = radio.read_register(RF_SETUP);
  delay(100);  
  radio.setPALevel( RF24_PA_HIGH );
  delay(100);  
  reg_value = radio.read_register(RF_SETUP);

//  reg_value = radio.read_register(SETUP_RETR);
  
}

void loop() {


  // radio.write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));
 
  // delay(100);  
  // // reg_value = radio.read_register(SETUP_RETR);
  // reg_value = radio.read_register(RF_SETUP);
  Serial.println(reg_value,HEX);
  delay(1000);

}








