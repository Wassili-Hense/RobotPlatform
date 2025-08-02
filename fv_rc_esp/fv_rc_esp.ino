#include "Wire.h"

// Joystick
#define I2C_DEV_ADDR 0x17
#define ANALOG_DELTA 15
static uint8_t jrBuff[6];
static uint8_t jdr = 0;
static uint16_t jaVal[] = {0, 0, 0};
static char jaName[] = {'V', 'X', 'Y'};
static uint8_t jbCnt[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//void onRequest() {
//  Wire.print(i++);
//  Wire.print(" Packets.");
//  Serial.println("onRequest");
//  Serial.println();
//}

void onJReceive(int len) {
  if(len==6){
    for(uint8_t i = 0; i<6; i++){
      jrBuff[i] = Wire.read();
    }
    jdr=1;
  }
}

int8_t onJAnalog(uint8_t idx){
  uint16_t val = (jrBuff[idx*2]<<4) | (jrBuff[idx*2+1]&0x0F);
  int32_t diff = val - jaVal[idx];
  if(diff>ANALOG_DELTA || diff<-ANALOG_DELTA){
    jaVal[idx] = val;
    return 1;
  }
  return 0;
}

int8_t onJDigital(uint8_t idx){
  uint8_t nv = (jrBuff[((idx&0x0C)>>1)+1]>>((idx&0x03)+4))&1;
  uint8_t ov = jbCnt[idx]>>7;
  uint8_t cnt = jbCnt[idx]&3;
  int8_t r=0;
  if(nv){
    if(ov){
      cnt=3;
    } else if(cnt<2){
      cnt++;
    } else {
      cnt = 3;
      ov = 1;
      r = 1;
    }
  } else {
    if(!ov){
      cnt = 0;
    }else if(cnt>1){
      cnt--;
    } else {
      cnt = 0;
      ov = 0;
      r = -1;
    }
  }
  jbCnt[idx] = (uint8_t)((ov<<7) | cnt);
  return r;
}
// Serial
static uint sOpen = false;

void setup() {
  Serial.end();
  Wire.onReceive(onJReceive);
  //Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
}

void loop() {
  if(jdr){
    jdr = 0;
    uint8_t i;
    int8_t r;
    for(i=0; i<3; i++){
      r = onJAnalog(i);
      if(r && sOpen){
        Serial.print(jaName[i]);
        Serial.print(": ");
        Serial.println(jaVal[i]);
      }
    }
    for(i=0; i<12; i++){
      r = onJDigital(i);
      if(!r) continue;
      if(sOpen){
        Serial.print("K: ");
        Serial.println((i+1)*r);
      }
    }
  } else {
    uint8_t usb = (uint8_t)(jbCnt[3]>>7);
    if(sOpen != usb){
      sOpen = usb;
      if(sOpen){
        Serial.begin(115200);
      } else {
        Serial.end();
      }
    }
    delay(1);
  }
}