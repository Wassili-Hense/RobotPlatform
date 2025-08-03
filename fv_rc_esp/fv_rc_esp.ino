#include "Wire.h"

// Joystick
#define I2C_DEV_ADDR 0x17
static uint8_t jrBuff[3];
static uint8_t jdr = 0;
static uint8_t usbOn = 0;
//void onRequest() {
//  Wire.print(i++);
//  Wire.print(" Packets.");
//  Serial.println("onRequest");
//  Serial.println();
//}

void onJReceive(int len) {
  if(len==3){
    Wire.readBytes(jrBuff, 3);
    jdr=1;
  }
}

// Serial
static uint sOpen = false;

void setup() {
  Serial.end();
  //Serial.begin(115200);
  Wire.onReceive(onJReceive);
  //Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
}

void loop() {
  if(jdr){
    jdr = 0;
    if(sOpen){
      Serial.print((char)jrBuff[0]);
      Serial.print(": ");
    }
    if(jrBuff[0]=='B'){
      if(sOpen){
        Serial.print(jrBuff[1]?'+':'-');
        Serial.println(jrBuff[2]);  
      }
    } else {
      if(sOpen){
        Serial.println((((uint16_t)jrBuff[1])<<8) | jrBuff[2]);
      }
      if(jrBuff[0]=='U'){
        usbOn = jrBuff[1] > 3;
      }
    }
  } else {
    if(sOpen != usbOn){
      sOpen = usbOn;
      if(sOpen){
        Serial.begin(115200);
      } else {
        Serial.end();
      }
    }
  }
  delay(1);
}