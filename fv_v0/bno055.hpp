#include <Wire.h>

#ifndef BNO055_hpp
#define BNO055_hpp

class BNO055 {
  public:
  BNO055(TwoWire *wire, bool addr = true);
  int8_t Initialize();
  int8_t Read();

  int16_t heading;
  int16_t roll;
  int16_t pitch;

  private:
  TwoWire *_wire;
  uint8_t _dev_addr;
  uint8_t _page;

  void WritePhase(uint16_t regaddr, bool et=true);
  uint8_t ReadRegister(uint16_t regaddr);
  uint16_t ReadRegister16(uint16_t regaddr);
  void WriteRegister(uint16_t regaddr, uint8_t value);
  
};
#endif  // BNO055_hpp