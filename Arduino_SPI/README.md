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

Видно, что регистр вренул нам 0x4F - то, что было записано