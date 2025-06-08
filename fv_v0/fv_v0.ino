//#include <Wire.h>
//#include "bno055.hpp"
#include "xiaomi_cybergear.h"

//BNO055 bno(&Wire, false);
TWAI twai = TWAI(/*RX_PIN=*/26, /*TX_PIN=*/25);
Cybergear cgL = Cybergear(&twai, 0x03);
Cybergear cgR = Cybergear(&twai, 0x04);

char _rxBuffer[17];
uint8_t _rxIdx;

void Command(char cmd, float val){
    int r;
    switch(cmd){
    case 'm':
      r=cgL.SetRunMode((int8_t)val);
      r=cgR.SetRunMode((int8_t)val);
    case 'p':
      r=cgL.Command(-val, 0, 0, 25, 3);
      r=cgR.Command(val, 0, 0, 25, 3);
      break;
    case 'c':
      r=cgL.SendFloat(0x7006, val);
      r=cgR.SendFloat(0x7006, -val);
      break;
    case 'v':
      r=cgL.SendFloat(0x700A, val);
      r=cgR.SendFloat(0x700A, -val);
      break;
    case 't':
      r=cgL.SendFloat(0x700B, val);
      break;
    case 'i':
      r=cgL.SendFloat(0x7018, val);
      break;
    case 'r':
      cgL.SetZero();
      r=cgL.Command(val, 0, 0, 8, 1);
      break;    
    default: 
      r=-33;
      break;
    }
    Serial.print(cmd);Serial.print("$");Serial.print(val);Serial.print("=");Serial.println(r);
}
void Terminal(){
  char c;
  while(Serial.available() > 0){
    c = (char)Serial.read();
    if(c=='\r'){
      String str = &_rxBuffer[1];
      Command(_rxBuffer[0], str.toFloat());
      _rxIdx = 0;
      _rxBuffer[0] = '\0';
    }else if (c == '\b' || c == 127) {
      if (_rxIdx > 0) {
         _rxIdx--;
         _rxBuffer[_rxIdx] = '\0';
      }
   } else if (isprint(c)) {
     // Store printable characters in serial receive buffer
     if (_rxIdx < 16) {
       _rxBuffer[_rxIdx++] = c;
       _rxBuffer[_rxIdx] = '\0';
     }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  //Wire.begin(); //Start I2C communication as master
  //Wire.setClock(400000);  //set I2C to fast mode at 400kHz

  //int8_t err = bno.Initialize();
  //if(err!=0){
  //  Serial.print("BNO055.Init - "); 
  //  Serial.println(err);
  //  while(1){
  //    delay(1000);
  //  }
  //}

  cgL.SetParameter(Cybergear_parameter_Limit_Current, 10.0f);
  cgL.SetParameter(Cybergear_parameter_Speed_kp, 1.2f);
  cgL.SetParameter(Cybergear_parameter_Speed_ki, 0.002f);
  cgL.SetRunMode(2);

  cgR.SetParameter(Cybergear_parameter_Limit_Current, 10.0f);
  cgR.SetParameter(Cybergear_parameter_Speed_kp, 1.2f);
  cgR.SetParameter(Cybergear_parameter_Speed_ki, 0.002f);
  cgR.SetRunMode(2);

  Serial.println("Ready");
}

void loop() {
  //bno.Read();
  //Serial.print(bno.heading/16.0);
  //Serial.print(", ");
  //Serial.print(bno.roll/16.0);
  //Serial.print(", ");
  //Serial.println(bno.pitch/16.0);
  //delay(100);
  int err = twai.Tick();
  if(err!=0){
    Serial.print("$");
    Serial.println(err);
    delay(1000);
    return;
  }
  delay(10);
  if(cgL.Tick()==1){
    uint8_t st = cgL.GetMotorStatus();
    Serial.print("v: ");Serial.println(cgL.speed);
    if(st&0x01){
      //Serial.println("Fault: Under Voltage");
      cgL.ClearFault();
    } 
  } else if(cgR.Tick()==1){
    uint8_t st = cgR.GetMotorStatus();
    if(st&0x01){
      //Serial.println("Fault: Under Voltage");
      cgR.ClearFault();
    } 
  } else {
    Terminal();
  }
}
