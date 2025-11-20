#ifndef NRF_HPP
#define NRF_HPP

#include <inttypes.h> //для uint8_t
#include "../inc/setup.hpp"
# include <libopencm3/stm32/rcc.h> 
# include <libopencm3/stm32/gpio.h> 

#define SETUP_RETR      0x04
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define REGISTER_MASK   0x1F
#define ARC             0
#define ARD             4




class RF24 {
public:
    RF24(uint16_t _cepin, uint32_t _ceport, uint16_t _cspin,uint32_t _csport);
    void csn(uint8_t mode);
    void ce(uint8_t level); 
    void begin(void);
    void write_register(uint8_t reg, uint8_t value);
    uint16_t read_register(uint8_t reg);


private:
  uint32_t ce_port;
  uint16_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
  uint32_t csn_port;
  uint16_t csn_pin; /**< SPI Chip select */

};

#endif