#ifndef NRF_HPP
#define NRF_HPP

#define HIGH 1
#define LOW  0

# include <libopencm3/stm32/rcc.h> 
# include <libopencm3/stm32/gpio.h> 

#include <inttypes.h> //для uint8_t
#include "../inc/setup.hpp"


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


class NRF24 {
public:
    NRF24(uint16_t _cepin, uint32_t _ceport, uint16_t _cspin,uint32_t _csport);
    void csn(int mode);
    void ce(int level); 
    void send_SPI (uint8_t data);
    uint8_t read_SPI ();
    uint8_t Transfer_SPI(uint8_t data);
    void enableDynamicPayloads(void);
    void writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);
    bool isAckPayloadAvailable(void);
    bool isPVariant(void);
    void setAutoAck( uint8_t pipe, bool enable );
    bool testRPD(void);
    bool testCarrier(void);
    rf24_pa_dbm_e getPALevel(void);
    rf24_datarate_e getDataRate( void );
    rf24_crclength_e getCRCLength(void);
    bool begin(void);
    uint8_t write_register(uint8_t reg, uint8_t value);
    uint8_t write_register(uint8_t reg, const uint8_t* buf, uint8_t len);
    uint8_t read_register(uint8_t reg);
    uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);
    void setPALevel(rf24_pa_dbm_e level);      //устанавливает мощность радиомодуля
    bool setDataRate(rf24_datarate_e speed);   // уставнавивает скорость передачи 
    bool setCRCLength(rf24_crclength_e length);// уставнавивает длинну CRC Суммы
    bool setChannel(uint8_t channel);
    uint8_t flush_rx(void);
    uint8_t flush_tx(void);

    // функции, в которых может быть ошибка - Миша,проверь!!
    void setAutoAck(bool enable);
    // void setAutoAck( uint8_t pipe, bool enable );
    void setRetries(uint8_t delay, uint8_t count);
    void enableAckPayload(void);
    void toggle_features(void);
    void setPayloadSize(uint8_t size);
    uint8_t getPayloadSize(void);
    void powerUp(void);
    void powerDown(void);
    void stopListening(void);
    void startListening(void);
    void startWrite( const void* buf, uint8_t len );
    uint8_t write_payload(const void* buf, uint8_t len);
    uint8_t read_payload(void* buf, uint8_t len);
    bool write( const void* buf, uint8_t len );
    bool read( void* buf, uint8_t len );
    void whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready);
    uint8_t getDynamicPayloadSize(void);
    bool available(void);
    bool available(uint8_t* pipe_num);
    uint8_t get_status(void);
    void openWritingPipe(uint64_t value); 
    void openReadingPipe(uint8_t child, uint64_t address);
    void disableCRC( void );

private:
  uint32_t ce_port;
  uint16_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
  uint32_t csn_port;
  uint16_t csn_pin; /**< SPI Chip select */
  bool wide_band; /* 2Mbs data rate in use? */
  bool p_variant; /* False for RF24L01 and true for RF24L01P */
  uint8_t payload_size; /**< Fixed size of payloads */
  bool ack_payload_available; /**< Whether there is an ack payload waiting */
  bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
  uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
  uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */

};

#endif