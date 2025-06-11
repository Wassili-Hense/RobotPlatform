#include <cstdint>
#include "TWAI.h"

#ifndef XIAOMI_CYBERGEAR_H
#define XIAOMI_CYBERGEAR_H

class Cybergear{
  public:
  float position;
  float speed;
  float torque;
  float temperature;

  Cybergear(TWAI *twai, uint8_t addr);
  // -1 - OFF
  //  0 - Operational Control Mode
  //  1 - Position mode
  //  2 - Velocity Mode
  //  3 - Current Mode
  int SetRunMode(int8_t run_mode);
  int Command(float target, float speed, float torque, float kp, float kd);
  int SetZero();
  //  1 - Status Updated
  int Tick();
  uint8_t GetMotorStatus(){ return _motorStatus; }
  int ClearFault();
  int SendFloat(uint16_t addr, float value);

  private:
  TWAI *_twai;
  uint8_t _addr;

  int8_t _runMode; // ADDR_RUN_MODE or -1
  uint8_t _motorStatus;

  uint32_t _to;
  int _waitUpdate;

  int StatusCB(uint32_t identifier, uint8_t length, uint8_t *data);
  int FaultCB(uint32_t identifier, uint8_t length, uint8_t *data);

  int SendRaw(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data);
};

#endif // XIAOMI_CYBERGEAR_H

