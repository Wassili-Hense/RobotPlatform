#include <functional>
#include <cstdint>

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

class TWAI_Msg{
  public:
    TWAI_Msg(uint32_t identifier, uint8_t length, uint8_t *data){
      _identifier = identifier;
      _length = length;
      for (int i = 0; i < length; i++) _data[i] = data[i];
    }
    uint32_t Identifier(){ return _identifier; }
    uint8_t Length(){ return _length; }
    uint8_t * Data(){ return _data; }
  
    TWAI_Msg *Next(){ return _next;}
    void Next(TWAI_Msg *r){ _next = r; }
  private:
    TWAI_Msg *_next = nullptr;
    uint32_t _identifier;
    uint8_t _length;
    uint8_t _data[8];
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

    TWAI_Msg *_mqHead;
    TWAI_Msg *_mqTail;

    int8_t Init();
};

#endif // TWAI_H