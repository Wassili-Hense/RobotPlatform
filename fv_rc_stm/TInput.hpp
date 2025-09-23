#include "wiring_time.h"
#include <cstdint>
#include <wiring.h>
#include <pins_arduino.h>
#include "Wire.h"

#ifndef T_INPUT_HPP
#define T_INPUT_HPP

class TInput;

using tsend_f_cb = void (*)(uint8_t, uint16_t);
typedef void (TInput::*ti_action)();
#define INP_ANALOG_TH 15

class TInput{
  private:
  uint8_t _id;
  uint8_t _rts;
  uint16_t _value;
  uint32_t _pin;
  uint32_t _to;
  ti_action _init;
  ti_action _tick;
  ti_action _send;

  void InitA(){
    pinMode(_pin, INPUT_ANALOG);
    _to = millis();
  }
  void TickA(){
    uint16_t value = (uint16_t)analogRead(_pin);
    int32_t diff = value - _value;
    int32_t td = millis() - _to;
    if(td>=100 && (diff>INP_ANALOG_TH || diff<-INP_ANALOG_TH || td>2450)){
      _rts = 1;
      _value = value;
    }
  }
  void SendA(){
    SendFunc(_id, _value);
    _to = millis();
  }
  void InitD(){
    pinMode(_pin, INPUT_PULLUP);
  }
  void TickD(){
    uint8_t ov = (uint8_t)(_value>>15);
    uint16_t st=(_value<<1) | (digitalRead(_pin)?0:1);
    if(st==(ov?0x8000:0x7FFF)){
      _rts = 1;
      ov = (uint8_t)(st & 1);
    }
    _value = (ov<<15) | (st & 0x7FFF);
  }
  void SendD(){
    SendFunc((uint8_t)'B', ((_value>>7)&0x100) | _id);
  }
  
  public:
  static tsend_f_cb SendFunc;

  TInput(bool analog, uint32_t pin, uint8_t id){
    _pin = pin;
    _id = id;
    if(analog){
      _init = &TInput::InitA;
      _tick = &TInput::TickA;
      _send = &TInput::SendA;
    } else {
      _init = &TInput::InitD;
      _tick = &TInput::TickD;
      _send = &TInput::SendD;
    }
  }
  void Init(){
    std::invoke(_init, this);
  }
  void Tick(){
    std::invoke(_tick, this);
  }
  int8_t Send(){
    if(_rts){
      _rts = 0;
      std::invoke(_send, this);
      return 1;
    }
    return 0;
  }
  uint16_t Value(){
    return _value;
  }
};

tsend_f_cb TInput::SendFunc;


#endif // T_INPUT_HPP