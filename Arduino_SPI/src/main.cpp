#include "../include/NRF.hpp"
#include <SPI.h>    

RF24 radio(9, 6); // "создать" модуль на пинах 9 и 6 




void setup() {
  // запустить шину
  SPI.begin();
  radio.begin();      

  
}

void loop() {
  radio.write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));
  delay(1000);
}








