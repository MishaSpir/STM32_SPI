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


class RF24 {
public:
    RF24(uint16_t _cepin, uint32_t _ceport, uint16_t _cspin,uint32_t _csport);
    void csn(uint8_t mode);
    void ce(uint8_t level); 
    void begin(void);
    void write_register(uint8_t reg, uint8_t value);
    uint16_t read_register(uint8_t reg);
    void setPALevel(rf24_pa_dbm_e level);   //устанавливает мощность радиомодуля


private:
  uint32_t ce_port;
  uint16_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
  uint32_t csn_port;
  uint16_t csn_pin; /**< SPI Chip select */

};

#endif