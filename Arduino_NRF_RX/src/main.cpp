#include "../include/NRF.hpp"
#include <SPI.h>    



RF24 radio(9, 6); // "создать" модуль на пинах 9 и 6 

uint8_t counter = 9;

uint8_t in_reg;
uint64_t tube = 0x314E6F6465;

void setup() {
  // запустить шину
  Serial.begin(9600);

  SPI.begin();
  pinMode(9,OUTPUT);
  pinMode(6,OUTPUT);

  // radio.begin(); 
  radio.write_register( RF_SETUP, 0x00 ) ;
	radio.setPALevel( RF24_PA_MAX ) ;
  radio.read_register(RF_SETUP ) ;
  

  // delay(1);
  // radio.setAutoAck(1);       //0x21 0x3F
  // delay(1);
  // radio.setRetries(0, 15);   //0x24 0x0F
  // delay(1);
  // radio.enableAckPayload();  //0x1D_0xFF  0x3D_0x06 0x1D_0xFF 0x1C_0xFF 0x3C_0x03  
  // delay(1);
  // radio.setPayloadSize(32);   // размер пакета, в байтах
  // delay(1);
  // // radio.openWritingPipe(address[0]);  // мы - труба 0, открываем канал для передачи данных
  // // radio.openWritingPipe(0x314E6F6465); // "1Node" в hex
  // radio.openReadingPipe(1,0x314E6F6465); // "1Node" в hex
  // delay(1);
  // radio.setChannel(0x60);             // выбираем канал (в котором нет шумов!)
  // delay(1);
  // radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  // delay(1);
  // radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  // //должна быть одинакова на приёмнике и передатчике!
  // delay(1);
  // radio.powerUp();        // начать работу
  // delay(1);
  // radio.startListening(); // начинаем слушать эфир, мы приёмный модуль

  // delay(5000);

  // byte pipeNo, gotByte;
  // while (radio.available(&pipeNo)) {        // слушаем эфир со всех труб
  //   radio.read(&gotByte, sizeof(gotByte));  // чиатем входящий сигнал
  //   Serial.print("Recieved: ");
  //   Serial.println(gotByte);
  // }
  
  // delay(1000);

 
}


void loop() {
  // delay(10);
  // radio.write(&counter, sizeof(counter));
  // counter++;
  // in_reg = (radio.read_register(SETUP_RETR));
  // Serial.println(in_reg,HEX);

  delay(100);
  radio.setPALevel( RF24_PA_MAX ) ;
  radio.read_register(RF_SETUP ) ;
}







