#include <functional>
#include <cstdint>
#include <driver/twai.h>
//https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html

#ifndef TWAI_H
#define TWAI_H

//typedef int(*TWAI_Callback)(uint32_t identifier, uint8_t length, uint8_t *data);

class TWAI_Sub{
  public:
    TWAI_Sub(uint32_t val, uint32_t mask, std::function<int8_t(uint32_t identifier, uint8_t length, uint8_t *data)> cb);
    int8_t Do(uint32_t identifier, uint8_t length, uint8_t *data);

    TWAI_Sub *next(){ return _next;}
    void next(TWAI_Sub *r){ _next = r; }

  private:
    uint32_t _value;
    uint32_t _mask;
    std::function<int8_t(uint32_t identifier, uint8_t length, uint8_t *data)> _cb;
    TWAI_Sub *_next = nullptr;
};

class TWAI{
  public:
    TWAI(uint8_t rx_pin, uint8_t tx_pin);
    ~TWAI();
    //  0 - Ok
    // -1 - Failed to install driver
    // -2 - Failed to start driver
    // -3 - Failed to reconfigure alerts
    // -4 - Failed to get twai status
    // -5 - Twai not runing
    // -6 - Twai controller has become error passive
    // -7 - Error has occurred on the bus
    // -8 - The transmission failed
    int8_t Tick();
    void Subscribe(uint32_t value, uint32_t mask, std::function<int8_t(uint32_t identifier, uint8_t length, uint8_t *data)> cb);
    //  0 - Ok
    // -9 - Failed to queue message for transmission
    int8_t Send(uint32_t identifier, uint8_t length, uint8_t *data);

  private:
    uint8_t _rx_pin;
    uint8_t _tx_pin;

    TWAI_Sub *_cbHead;
    TWAI_Sub *_cbTail;
    uint8_t _mqHead;
    uint8_t _mqTail;
    uint8_t _mqCount;
    twai_message_t _mqArr[16];
    int8_t Init();
};

#endif // TWAI_H