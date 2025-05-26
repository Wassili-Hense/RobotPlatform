#include <functional>
#include <limits>
#include <map>
#include <tuple>
#include <esp32-hal.h>
//#include <HardwareSerial.h>
#include "xiaomi_cybergear_defs.h"
#include "xiaomi_cybergear.h"

uint8_t MASTER_CAN_ID = 0x00;
uint32_t STATUS_PERIODE_MS = 50; 

std::map<uint16_t, std::tuple<float, float>> CG_Parameters = {
  {ADDR_SPEED_REF, {-V_MAX, V_MAX}},
  {ADDR_LIMIT_TORQUE, {0, T_MAX}},
  {ADDR_LIMIT_SPEED, {0, V_MAX}},
  {ADDR_LIMIT_CURRENT, {0, I_MAX}}
};

float ushort2float(uint16_t x, float min, float max){
  return (float) x / std::numeric_limits<uint16_t>::max() * (max - min) + min;
}
uint16_t float2ushort(float x, float min, float max){
  if(x > max) return std::numeric_limits<uint16_t>::max();
  if(x < min) return 0;
  return (uint16_t) ((x-min)*((float)std::numeric_limits<uint16_t>::max())/(max - min));
}

int findByAddr(std::vector<std::tuple<uint16_t, float>> vec, uint16_t addr){
  for(int i=0; i<vec.size(); i++) if(std::get<0>(vec[i])==addr) return i;
  return -1;
}

Cybergear::Cybergear(TWAI *twai, uint8_t addr){
  using namespace std::placeholders;

  _twai = twai;
  _addr=addr;
  _motorStatus = 0;
  _runMode = -1;
  _paramIdx=0;
  _twai->Subscribe((CMD_REQUEST<<24) | (((uint32_t)_addr)<<8), 0x1F00FF00, std::bind(&Cybergear::StatusCB, this, _1, _2, _3));
  _twai->Subscribe((CMD_FEEDBACK<<24) | (((uint32_t)_addr)<<8), 0x1F00FF00, std::bind(&Cybergear::FaultCB, this, _1, _2, _3));
}

int Cybergear::SetRunMode(int8_t run_mode){
  _runMode = run_mode;
  if(run_mode<0){
    uint8_t data[8] = {0x00};
    return SendRaw(_addr, CMD_STOP, MASTER_CAN_ID, 8, data);
  }
  uint8_t data[8] = {0x00};
  data[0] = ADDR_RUN_MODE & 0x00FF;
  data[1] = ADDR_RUN_MODE >> 8;
  data[4] = _runMode;
  return SendRaw(_addr, CMD_RAM_WRITE, MASTER_CAN_ID, 8, data);
}

int Cybergear::SetParameter(Cybergear_parameter pAddr, float value){
  uint16_t addr = (uint16_t)pAddr;
  if(!CG_Parameters.contains(addr)) return -10;
  float min, max;
  std::tie(min, max) = CG_Parameters[addr];
  if(value<min) value = min;
  if(value>max) value = max;

  int i = findByAddr(_parameters, addr);
  if(i<0){
    _parameters.push_back(std::tuple<uint16_t, float>{addr, value});
    if(_twai->RTS()) _paramIdx = _parameters.size();
  } else{
    _parameters.at(i) = std::tuple<uint16_t, float>{addr, value};
  }
  return SendFloat(addr, value);
}

int Cybergear::Command(float position, float speed, float torque, float kp, float kd){
  uint8_t data[8] = {0x00};

  uint16_t tmp = float2ushort(position, -POS_MAX, POS_MAX);
  data[0] = tmp >> 8;
  data[1] = tmp & 0x00FF;

  tmp = float2ushort(speed, -V_MAX, V_MAX);
  data[2] = tmp >> 8;
  data[3] = tmp & 0x00FF;

  tmp = float2ushort(kp, KP_MIN, KP_MAX);
  data[4] = tmp >> 8;
  data[5] = tmp & 0x00FF;

  tmp = float2ushort(kd, KD_MIN, KD_MAX);
  data[6] = tmp >> 8;
  data[7] = tmp & 0x00FF;

  tmp = float2ushort(torque, -T_MAX, T_MAX);
  return SendRaw(_addr, CMD_CONTROL, tmp, 8, data);
}
int Cybergear::SetZero(){
    uint8_t data[8] = {0x00};
    data[0]=1;
    return SendRaw(_addr, CMD_SET_MECH_POSITION_TO_ZERO, MASTER_CAN_ID, 8, data);
}
int Cybergear::Tick() {
  if(_twai->RTS()){
    uint32_t cur_t = millis();
    if((cur_t-_to >= STATUS_PERIODE_MS)){
      _to = cur_t;
      _waitUpdate = 0;
      uint8_t data[8] = {0x00};
      int r=SendRaw(_addr, CMD_REQUEST, MASTER_CAN_ID, 8, data);
      if(r!=0) return r;
    } else if(_runMode>=0 && (_motorStatus & 0xC0)!=0x80){
      if(_paramIdx < _parameters.size()){
        uint16_t addr;
        float value;
        std::tie(addr, value)=_parameters.at(_paramIdx);
        _paramIdx++;
        int r = SendFloat(addr, value);
        if(r!=0) return r;
      } else {
        uint8_t data[8] = {0x00};
        int r = SendRaw(_addr, CMD_ENABLE, MASTER_CAN_ID, 8, data);      
        if(r!=0) return r;
      }
    } else if(_runMode<0 && (_motorStatus & 0xC0)==0x80){
      uint8_t data[8] = {0x00};
      int r = SendRaw(_addr, CMD_STOP, MASTER_CAN_ID, 8, data);
      if(r!=0) return r;
    }
  }

  if(_waitUpdate==1){
    _waitUpdate=2;
    return 1;
  }
  return 0;
}

int Cybergear::ClearFault(){
  _to=0;
  uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
  return SendRaw(_addr, (_motorStatus&0xC0)==0x80?CMD_ENABLE:CMD_STOP, MASTER_CAN_ID, 8, data);
}

int Cybergear::StatusCB(uint32_t identifier, uint8_t length, uint8_t *data){
  if((identifier&0xC00000)!=0x800000 && (_motorStatus & 0xC0)==0x80){
    _paramIdx = 0;
  }
  _motorStatus = (uint8_t)(identifier>>16);
  position = ushort2float((uint16_t)data[1] | data[0] << 8, -POS_MAX, POS_MAX);
  speed = ushort2float((uint16_t)data[3] | data[2] << 8, -V_MAX, V_MAX);
  torque = ushort2float((uint16_t)data[5] | data[4] << 8, -T_MAX, T_MAX);
  temperature = (float)(data[7] | data[6] << 8)/10.0f;
  _waitUpdate = 1;
  return 0;
}

int Cybergear::FaultCB(uint32_t identifier, uint8_t length, uint8_t *data){
  //Serial.print("!!");
  //for(int i = length-1; i>=0; i--){
  //  Serial.print(data[i]>>4, HEX);
  //  Serial.print(data[i]&0x0F, HEX);
  //}
  //Serial.println();
  return 0;
}

int Cybergear::SendRaw(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data){
    uint32_t id = cmd_id << 24 | option << 8 | can_id;
    return _twai->Send(id, len, data);
}

int Cybergear::SendFloat(uint16_t addr, float value){
   uint8_t data[8];
   data[0] = addr & 0x00FF;
   data[1] = addr >> 8;
   data[2] = 0;
   data[3] = 0;
   memcpy(&data[4], &value, 4);

   int r=SendRaw(_addr, CMD_RAM_WRITE, MASTER_CAN_ID, 8, data);
   //Serial.printf("sendFloat(%x, %f) - %d\n", addr, value, r);
   return r;
}