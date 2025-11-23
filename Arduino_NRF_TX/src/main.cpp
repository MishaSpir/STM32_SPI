#include "../include/NRF.hpp"
#include <SPI.h>    


byte counter = 8;
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб

RF24 radio(9, 6); // "создать" модуль на пинах 9 и 6 

bool p_var = 0;
uint8_t reg_value;

void setup() {
  // запустить шину
  Serial.begin(9600);

  SPI.begin();
  radio.begin();  
  delay(1);
  radio.setAutoAck(1);       //0x21 0x3F
  delay(1);
  radio.setRetries(0, 15);   //0x24 0x0F
  delay(1);
  radio.enableAckPayload();  //0x1D_0xFF  0x3D_0x06 0x1D_0xFF 0x1C_0xFF 0x3C_0x03  
  delay(1);
  radio.setPayloadSize(32);   // размер пакета, в байтах

  // radio.openWritingPipe(address[0]);  // мы - труба 0, открываем канал для передачи данных
  radio.openWritingPipe(0x314E6F6465); // "1Node" в hex
  radio.setChannel(0x60);             // выбираем канал (в котором нет шумов!)

  delay(1);
  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  delay(1);
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!

  delay(1);
  radio.powerUp();        // начать работу
  delay(1);
  radio.stopListening();  // не слушаем радиоэфир, мы передатчик

  
}

void loop() {

  Serial.print("Sent: ");
  Serial.println(counter);
  radio.write(&counter, sizeof(counter));
  counter++;
  delay(2000);

}








