# передача данных по SPI
## pinout
![alt text](image-1.png)

![alt text](image-2.png)




![alt text](image.png)
**результат работы функции:** 
`RF24_write_register(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));`

```
#define SETUP_RETR  0x04
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ARC         0
#define ARD         4
```