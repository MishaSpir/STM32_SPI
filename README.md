# переписываем библиотеку(GyverNRF) c Arduino нас STM32
## write_register
**- на Arduino:**
![alt text](pictures/arduino_out_read_register.png)

**- на STM32_103:**

![alt text](pictures/stm103_out_write_register.png)



## read_register

**- на Arduino:**

![alt text](pictures/arduino_out_read_register.png)

**- на STM32_103:**

![alt text](pictures/stm103_out_read_register.png)

Видно, что второе значение по MOSI из Arduino и из STM103 не совпадают, но это неважно, так как оно послылается, чтобы считать значение из регистра радиомодуля. А оно, как можно видеть для Arduino и для STM103 одинаковое (0x4F).


## setPALevel

**- на Arduino:**

![alt text](pictures/Arduino_SPI/powerHigh.png)

**- на STM32_103:**

![alt text](pictures/powerHighstm.png)

функция записывает мощность в регистр. Подробнее читай в README в папке ARDUINO_SPI
