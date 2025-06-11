#include <Wire.h>
#include "bno055.hpp"
#include "xiaomi_cybergear.h"

#define BNO_INT 16
BNO055 bno(&Wire, false);
TWAI twai = TWAI(/*RX_PIN=*/26, /*TX_PIN=*/25);
//Cybergear cgL = Cybergear(&twai, 0x03);
//Cybergear cgR = Cybergear(&twai, 0x04);

uint32_t bno_to;
//float val;
//float kp;

/*
void Command(char cmd, float val){
    int r;
    switch(cmd){
    case 'm':
      r=cgL.SetRunMode((int8_t)val);
      r=cgR.SetRunMode((int8_t)val);
    case 'v':
      r=cgL.SendFloat(0x700A, val);
      r=cgR.SendFloat(0x700A, -val);
      break;
    case 'k':
      r=0;
      kp = val;
      break;
    default: 
      r=-33;
      break;
    }
    Serial.print(cmd);Serial.print("$");Serial.print(val);Serial.print("=");Serial.println(r);
}

char _rxBuffer[17];
uint8_t _rxIdx;

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
}*/

float rollPrev;
void setup() {
  //val=0;
  //kp = 0.2;
  Serial.begin(115200);
  Serial.println("Start");

  Wire.begin(); //Start I2C communication as master
  Wire.setClock(400000);  //set I2C to fast mode at 400kHz
  pinMode(BNO_INT, INPUT);
  
  int8_t err;
  do{
    err = bno.Initialize();
    if(err!=0){
      Serial.print("BNO055.Init - "); 
      Serial.println(err);
      delay(1000);
    }
  }while(err!=0);
  do{
    err=twai.Tick();
    if(err!=0){
      Serial.print("TWAI.Tick - ");
      Serial.println(err);
      delay(1000);
    }
  }while(err!=0);
  //cgL.SetParameter(Cybergear_parameter_Limit_Current, 20.0f);
  //cgL.SetParameter(Cybergear_parameter_Speed_kp, 1.2f);
  //cgL.SetParameter(Cybergear_parameter_Speed_ki, 0.002f);
  //cgL.SetRunMode(2);

  //cgR.SetParameter(Cybergear_parameter_Limit_Current, 20.0f);
  //cgR.SetParameter(Cybergear_parameter_Speed_kp, 1.2f);
  //cgR.SetParameter(Cybergear_parameter_Speed_ki, 0.002f);
  //cgR.SetRunMode(2);

  Serial.println("Ready");
}

void loop() {
  uint32_t cur_t = millis();
  uint8_t st;
  
  st = twai.Tick();
  if(st!=0){
    Serial.print("$");
    Serial.println(st);
    delay(1000);
    return;
  }
  
  if((cur_t - bno_to >= 10)){
    bno_to = cur_t;
    //Serial.print(bno.GetRoll()/16.0);Serial.println();
  }

  /*
  if(cgL.Tick()==1){
    st = cgL.GetMotorStatus();
    if(st&0x01){
      //Serial.println("Fault: Under Voltage");
      cgL.ClearFault();
    } else if((st&0xC0) == 0x80){
      cgL.SendFloat(0x700A, val);
    }
  } else if(cgR.Tick()==1){
    st = cgR.GetMotorStatus();
    if(st&0x01){
      //Serial.println("Fault: Under Voltage");
      cgR.ClearFault();
    } else if((st&0xC0) == 0x80){
      cgR.SendFloat(0x700A, -val);
    }
  } else {
    Terminal();
  }*/
}
