#include "xiaomi_cybergear.h"

TWAI twai = TWAI(/*RX_PIN=*/4, /*TX_PIN=*/5);
Cybergear cg = Cybergear(&twai, 0x7F);
char _rxBuffer[17];
uint8_t _rxIdx;


void terminal(){
  char c;
  while(Serial.available() > 0){
    c = (char)Serial.read();
    if(c=='\r'){
      String str = &_rxBuffer[1];
      float val = str.toFloat();
      int r;
      switch(_rxBuffer[0]){
        case 'p':
        r = cg.Command(val, 0, 0, 25, 3);
        if(r==0){
          Serial.print("+");
        } else {
          Serial.print(r);
          Serial.print("!");
        }
        break;
        case 'v':
        r = cg.SendFloat(0x7017, val);
        if(r==0){
          Serial.print("+");
        } else {
          Serial.print(r);
          Serial.print("!");
        }
        break;
        case 'f':
        r = cg.SendFloat(0x700B, val);
        if(r==0){
          Serial.print("+");
        } else {
          Serial.print(r);
          Serial.print("!");
        }
        break;
        case 'r':
        r = cg.SetZero();
        r = cg.Command(val, 0, 0, 8, 1);
        if(r==0){
          Serial.print("+");
        } else {
          Serial.print(r);
          Serial.print("!");
        }
        break;
        default:
        Serial.print("-");
      }
      Serial.println(_rxBuffer);
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
  cg.SetParameter(Cybergear_parameter_Limit_Current, 25.0f);
  cg.SetRunMode(0);
}

void loop() {
  int err = twai.Tick();
  if(err!=0){
    Serial.print("$");
    Serial.println(err);
    delay(1000);
    return;
  }
  delay(100);
  if(cg.Tick()==1){
    uint8_t st = cg.GetMotorStatus();
    if(st&0x01){
      Serial.println("Fault: Under Voltage");
      cg.ClearFault();
    } 
    if(st&0x02) Serial.println("Fault: Overcurrent");
    if(st&0x04) Serial.println("Fault: over Temperature");
    if(st&0x08) Serial.println("Fault: encoder");
    if(st&0x10) Serial.println("Fault: Hall");
    if(st&0x20) Serial.println("Not calibrated");
    Serial.printf("M:%x, P:%f, V:%f, F:%f, T:%f\n", st>>6, cg.position, cg.speed, cg.torque, cg.temperature);
  }
  if((cg.GetMotorStatus()&0xC0)==0x80){
    terminal();
  }
}
