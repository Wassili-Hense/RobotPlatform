#include <Wire.h>
#include "bno055.hpp"
#include "xiaomi_cybergear.h"

#define BNO_INT 16
BNO055 bno(&Wire, false);
TWAI twai = TWAI(/*RX_PIN=*/26, /*TX_PIN=*/25);
Cybergear cgL = Cybergear(&twai, 0x03);
Cybergear cgR = Cybergear(&twai, 0x04);

float kp;


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
}

float rollPrev;
void setup() {
  kp = 0.5;
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
  Serial.print("bno.FwRev = ");Serial.println(bno.GetFwRev(), HEX);

  do{
    err=twai.Tick();
    if(err!=0){
      Serial.print("TWAI.Tick - ");
      Serial.println(err);
      delay(1000);
    }
  }while(err!=0);

  cgL.SendFloat(/*Cybergear_parameter_Limit_Current*/0x7018, 10.0f);
  cgR.SendFloat(/*Cybergear_parameter_Limit_Current*/0x7018, 10.0f);
  cgL.SetRunMode(2);
  cgR.SetRunMode(2);
  cgL.Enable();
  cgR.Enable();

  //cgR.SetParameter(Cybergear_parameter_Limit_Current, 20.0f);
  //cgR.SetParameter(Cybergear_parameter_Speed_kp, 1.2f);
  //cgR.SetParameter(Cybergear_parameter_Speed_ki, 0.002f);
  //cgR.SetRunMode(2);

  Serial.println("Ready");
}

void loop() {
  int8_t err;
  uint8_t st;
  err = twai.Tick();
  if(err!=0){
    Serial.print("$");
    Serial.println(err);
    delay(1000);
    return;
  }

  if(digitalRead(BNO_INT) && (bno.GetIntSta()&1)!=0){
    //Serial.print(bno.GetRoll()/16.0);Serial.println();
    float r = bno.GetRoll()/16.0;
    //float v = sqrt(abs(r))*kp*(r<0?-1:1);
    float v = r*kp;
    if((cgL.GetMotorStatus()&0xC0) == 0x80){
      cgL.SendFloat(0x700A, v);
    }    
    if((cgR.GetMotorStatus()&0xC0) == 0x80){
      cgR.SendFloat(0x700A, -v); 
    }    
  }
  
  Terminal();

  
  /*if(cgL.Tick()==1){
    st = cgL.GetMotorStatus();
    if(st&0x01){
      //Serial.println("Fault: Under Voltage");
      cgL.ClearFault();
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
