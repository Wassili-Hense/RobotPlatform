#include <esp_now.h>
#include <WiFi.h>
#include "xiaomi_cybergear.h"

TWAI twai = TWAI(/*RX_PIN=*/4, /*TX_PIN=*/5);
Cybergear cg = Cybergear(&twai, 0x7F);

char _rxBuffer[17];
uint8_t _rxIdx;

struct message {
  char cmd;
  float val;
};

float p_set, f_set=2;

void Command(char cmd, float val){
    int r;
    switch(cmd){
    case 'p':
      r=cg.Command(-val, 0, 0, 25, 3);
      p_set = val;
      break;
    case 'v':
      r=cg.SendFloat(0x7017, val);
      break;
    case 't':
      r=cg.SendFloat(0x700B, val);
      f_set=val;
      break;
    case 'r':
      cg.SetZero();
      r=cg.Command(val, 0, 0, 8, 1);
      break;    
    default: 
      r=-33;
      break;
    }
    //Serial.print("$");Serial.print(msg.cmd);Serial.print("$");Serial.print(msg.val);
    //Serial.print("=");Serial.println(r);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(len == sizeof(message) && (cg.GetMotorStatus()&0xC0)==0x80){
    message msg;
    memcpy(&msg, incomingData, sizeof(message));
    Command(msg.cmd, msg.val);
  }
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
  cg.SetParameter(Cybergear_parameter_Limit_Current, 25.0f);
  cg.SetRunMode(0);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("Ready");
}

void loop() {
  int err = twai.Tick();
  if(err!=0){
    Serial.print("$");
    Serial.println(err);
    delay(1000);
    return;
  }
  delay(10);
  if(cg.Tick()==1){
    uint8_t st = cg.GetMotorStatus();
    if(st&0x01){
      //Serial.println("Fault: Under Voltage");
      cg.ClearFault();
    } 
    //if(st&0x02) Serial.println("Fault: Overcurrent");
    //if(st&0x04) Serial.println("Fault: over Temperature");
    //if(st&0x08) Serial.println("Fault: encoder");
    //if(st&0x10) Serial.println("Fault: Hall");
    //if(st&0x20) Serial.println("Not calibrated");
    //Serial.printf("M:%x, P:%f, V:%f, F:%f, T:%f\n", st>>6, cg.position, cg.speed, cg.torque, cg.temperature);
    Serial.printf("Position:%f:%f, Torque:%f:%f:%f\n", -cg.position, p_set, -cg.torque, -f_set, f_set);
  } else {
    Terminal();
  }
}
