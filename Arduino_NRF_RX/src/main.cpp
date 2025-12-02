#include "../include/NRF.hpp"
#include <SPI.h>    

RF24 radio(9, 6); // "создать" модуль на пинах 9 и 6 




void setup() {
  Serial.begin(9600);         // открываем порт для связи с ПК
   delay(1);
  radio.begin();              // активировать модуль
   delay(1);
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
   delay(1);
  radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
   delay(1);
  // radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
   delay(1);
  radio.setPayloadSize(32);   // размер пакета, в байтах

  radio.openReadingPipe(1, 0x314E6F6465);  // "1Node" в hex
  radio.setChannel(0x70);     // выбираем канал (в котором нет шумов!)

   delay(1);
  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
   delay(1);
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp();        // начать работу
   delay(1);
  radio.startListening(); // начинаем слушать эфир, мы приёмный модуль
}

void loop() {
  uint8_t pipeNo, gotByte;
  	
    if (radio.available(&pipeNo)) {
      radio.write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
      // radio.write_register(STATUS, _BV(MAX_RT) | _BV(TX_DS) | _BV(RX_DR));
        radio.read(&gotByte, sizeof(gotByte));
            // radio.ce(0);
        Serial.println(gotByte);
    delay(10);
    // radio.ce(1);
    }
}