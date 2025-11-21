## pinout

```
CLK     D13
MOSI    D11
MISO    D12
CS      D6
CE      D9

```

![alt text](image-2.png)

#### Попробуем записать данные в регистр и считать данные из регистра
**Запись 0x4F в регистр SETUP_RETR:**
 
![alt text](image.png)


**Чтение данных из регистра SETUP_RETR:**

![alt text](image-1.png)

Видно, что регистр вернул нам 0x4F - то, что было записано


#### Сконфигурируем мощность радиомодуля
в документации сказано 

![alt text](image-3.png)



Значит, мощности записываются в регистр RF_PWR в биты 2:1
```
11 - MAX
10 - HIGH
01 - LOW
00 - MIN

```
попробуем записать мощность  HIGH
**читаем регистр RF_SETUP:**

![alt text](powerHigh.png)


попробуем записать мощность  LOW
**читаем регистр RF_SETUP:**

![alt text](powerLow.png)



