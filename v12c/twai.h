#include <functional>
#include <cstdint>

#ifndef TWAI_H
#define TWAI_H

typedef int(*TWAI_Callback)(uint32_t identifier, uint8_t length, uint8_t *data);

class TWAI_Sub{
  public:
    TWAI_Sub(uint32_t val, uint32_t mask, TWAI_Callback cb);
    int Do(uint32_t identifier, uint8_t length, uint8_t *data);

    TWAI_Sub *next(){ return _next;}
    void next(TWAI_Sub *r){ _next = r; }

  private:
    uint32_t _value;
    uint32_t _mask;
    TWAI_Callback _cb;
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
    int Tick();
    void Subscribe(uint32_t value, uint32_t mask, TWAI_Callback cb);
    //  0 - Ok
    // -9 - Failed to queue message for transmission
    int Send(uint32_t identifier, uint8_t length, uint8_t *data);
    bool TxEmpty(){ return _tx_empty; }

  private:
    uint8_t _rx_pin;
    uint8_t _tx_pin;

    TWAI_Sub *_cbHead;
    TWAI_Sub *_cbTail;

    bool _tx_empty;

    int Init();
};

#endif // TWAI_H