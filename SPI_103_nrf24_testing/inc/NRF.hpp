#ifndef NRF_HPP
#define NRF_HPP

# include <libopencm3/stm32/rcc.h> 
# include <libopencm3/stm32/gpio.h> 

#include <inttypes.h> //для uint8_t
#include "../inc/setup.hpp"
#include "../inc/R24_config.h"
#include "../inc/nrf24l01.h"


typedef enum {RF24_PA_MIN = 0, // Мин мощность,ток - 7,5 мА
              RF24_PA_LOW,     // Низкая мощность, ток - 9 мА
              RF24_PA_HIGH,    // Срдняя мощность, ток - 11 мА
              RF24_PA_MAX,     // Макс мощность, ток - 13,5 мА
              RF24_PA_ERROR 
             } rf24_pa_dbm_e ;

typedef enum { RF24_1MBPS = 0, // 0 - скорость 1 Мбит/с
               RF24_2MBPS,     // 1 - скорость 2 Мбит/с (автоматически = 1)
              RF24_250KBPS     // 2 - скорость 250 Кбит/с (автоматически = 2)
             } rf24_datarate_e;             

typedef enum {
               RF24_CRC_DISABLED = 0,  // 0 - CRC отключен
               RF24_CRC_8,             // 1 - CRC 8 бит (автоматически = 1)
               RF24_CRC_16             // 2 - CRC 16 бит (автоматически = 2)
              } rf24_crclength_e;


class RF24 {
public:
    RF24(uint16_t _cepin, uint32_t _ceport, uint16_t _cspin,uint32_t _csport);
    void csn(uint8_t mode);
    void ce(uint8_t level); 
    void begin(void);
    void write_register(uint8_t reg, uint8_t value);
    uint16_t read_register(uint8_t reg);
    void setPALevel(rf24_pa_dbm_e level);      //устанавливает мощность радиомодуля
    bool setDataRate(rf24_datarate_e speed);   // уставнавивает скорость передачи 
    void setCRCLength(rf24_crclength_e length);// уставнавивает длинну CRC Суммы
    void setChannel(uint8_t channel);
    void flush_rx(void);
    void flush_tx(void);

private:
  uint32_t ce_port;
  uint16_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
  uint32_t csn_port;
  uint16_t csn_pin; /**< SPI Chip select */
  bool wide_band; /* 2Mbs data rate in use? */
  bool p_variant; /* False for RF24L01 and true for RF24L01P */

};

#endif