#ifndef NRF_HPP
#define NRF_HPP

#include <Arduino.h>
#include <SPI.h>

#define SETUP_RETR      0x04
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define REGISTER_MASK   0x1F
#define ARC             0
#define ARD             4




class RF24 {
public:
    RF24(uint8_t _cepin, uint8_t _cspin);
    void csn(int mode);
    void ce(int level);
    void begin(void);
    uint8_t write_register(uint8_t reg, uint8_t value);
    uint16_t read_register(uint8_t reg);


private:
  uint32_t ce_port;
  uint16_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
  uint32_t csn_port;
  uint16_t csn_pin; /**< SPI Chip select */

};

#endif