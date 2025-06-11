#include <sys/unistd.h>
#include "bno055.hpp"
#include "bno055_reg.h"

BNO055::BNO055(TwoWire *wire, bool addr){
  _wire = wire;
  _dev_addr = addr ? BNO055_I2C_ADDR2 : BNO055_I2C_ADDR1;
  _page = 0xFF;
}
int8_t BNO055::Initialize(){
  if(ReadRegister(BNO055_CHIP_ID_ADDR)!=0xA0) return -7;
  if((ReadRegister(BNO055_OPR_MODE_ADDR)&0x0F)!=BNO055_OPERATION_MODE_CONFIG){
    WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
    delay(20);  // Switching time any operation mode -> config mode 19ms (Table 3-6)
  }
  WriteRegister(BNO055_UNIT_SEL_ADDR, 0x80);
  WriteRegister(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
  delay(8);  // Switching time config mode -> any operation mode 7ms (Table 3-6)
  
  uint8_t st = ReadRegister(BNO055_SYS_STAT_ADDR);
  if(st==5) return 0;
  return ReadRegister(BNO055_SYS_ERR_ADDR);
}
int16_t BNO055::GetHeading(){
  return ReadRegister16(BNO055_EULER_H_LSB_ADDR);
}
int16_t BNO055::GetRoll(){
  return ReadRegister16(BNO055_EULER_R_LSB_ADDR);
}
int16_t BNO055::GetPitch(){
  return ReadRegister16(BNO055_EULER_P_LSB_ADDR);
}
/***** Private Functions *****/

inline void BNO055::WritePhase(uint16_t regaddr, bool et){
  uint8_t page = regaddr >> 8;

  if(_page!=page){
    _wire->beginTransmission(_dev_addr);
    _wire->write(BNO055_PAGE_ID_ADDR);
    _wire->write(page);
    _wire->endTransmission();
    _page = page;
  }
  _wire->beginTransmission(_dev_addr);
  _wire->write(regaddr);
  if(et){
    _wire->endTransmission();
  }
}

inline uint8_t BNO055::ReadRegister(uint16_t regaddr){	//reads byte from a register
  uint8_t value = 0;
  WritePhase(regaddr);
  _wire->requestFrom(_dev_addr, 1, (int)true);
  while(_wire->available() < 1);
  value = _wire->read();
  return value;
}

inline uint16_t BNO055::ReadRegister16(uint16_t regaddr){
  uint16_t value = 0;
  uint8_t tmp = 0;
  WritePhase(regaddr);
  _wire->requestFrom(_dev_addr, 2, (int)true);
  while(_wire->available() < 2);
  tmp = _wire->read();
  value = (_wire->read()<<8) | tmp;
  return value;
}

inline void BNO055::WriteRegister(uint16_t regaddr, uint8_t value){	//writes byte to a register
  WritePhase(regaddr, false);
  Wire.write(value);
  Wire.endTransmission();
}
