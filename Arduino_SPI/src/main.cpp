
#include "../include/NRF.h"





void setup() {
  // запустить шину
  SPI.begin();

  // пин CS как выход
  pinMode(spi_ss, OUTPUT);

  // деактивировать
  digitalWrite(spi_ss, HIGH);

  delay(100);

  // отправить данные
  sendByte('A');
}

void loop() {
  RF24_write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));
  delay(1000);
}








