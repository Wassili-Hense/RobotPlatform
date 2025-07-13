#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "bno055.hpp"
#include "pid.h"
#include "xiaomi_cybergear.h"

#define BNO_INT 16

BNO055 bno(&Wire, false);
TWAI twai = TWAI(/*RX_PIN=*/26, /*TX_PIN=*/25);
Cybergear cgL = Cybergear(&twai, 0x03);
Cybergear cgR = Cybergear(&twai, 0x04);

float tVelocity = 0;
float turn = 0;
float VelI = 40*0.001;
float stabK = 0.5;
PIDController AnglePID(30, 500, 0.55, 0, 10);
PIDController VelocityPID(1.5*0.001, VelI, 0.01*0.001, 0, 0.3);

uint8_t tgtAddr[] = {0x08, 0xD1, 0xF9, 0x38, 0x2C, 0x70};
esp_now_peer_info_t peerInfo;
uint8_t nowAnsw[] = {0x11, 0};

int8_t Command(char cmd, float value){
    int r;
    switch(cmd){
    case 'p':
      AnglePID.P = value;
      r=0;
      break;
    case 'i':
      AnglePID.I = value;
      r=0;
      break;
    case 'd':
      AnglePID.D = value;
      r=0;
      break;
    case 'k':
      stabK = value;
      r=0;
      break;
    case 'P':
      VelocityPID.P = value*0.001;
      r=0;
      break;
    case 'I':
      VelI = value*0.001;
      //VelocityPID.I = value*0.001;
      r=0;
      break;
    case 'D':
      VelocityPID.D = value*0.001;
      r=0;
      break;
    case 'v':
      r=0;
      tVelocity = value;
      break;
    case 'r':
      r=0;
      turn = -value*0.5;
      break;
    default: 
      r=-33;
      break;
    }
    return r;
}

char _rxBuffer[17];
uint8_t _rxIdx;

void Terminal(){
  char c;
  while(Serial.available() > 0){
    c = (char)Serial.read();
    if(c=='\r'){
      String str = &_rxBuffer[1];
      float val = str.toFloat();
      int8_t r = Command(_rxBuffer[0], val);
      Serial.print(_rxBuffer[0]);Serial.print("$");Serial.print(val);Serial.print("=");Serial.println(r);
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

struct nMsg {
  char cmd;
  float val;
};

uint8_t _sndBuff[sizeof(nMsg)];

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(len == sizeof(nMsg)){
    nMsg msg;
    memcpy(&msg, incomingData, sizeof(nMsg));
    int8_t r = Command(msg.cmd, msg.val);
    nowAnsw[1] = r;
    esp_now_send(tgtAddr, nowAnsw, 2);
  }
}
void SendNMsg(char cmd, float val){
  nMsg msg;
  msg.cmd = cmd;
  msg.val = val;
  memcpy(_sndBuff, &msg, sizeof(nMsg));
  esp_now_send(tgtAddr, _sndBuff, sizeof(nMsg));
}


void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  } else {
    Serial.println("Error initializing ESP-NOW");
  }
  // Register peer
  memcpy(peerInfo.peer_addr, tgtAddr, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }

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

  bool l = false;
  bool r = false;
  do{
    err=twai.Tick();
    if(err!=0){
      Serial.print("T");
      Serial.println(err);
      delay(1000);
      continue;
    }
    if(cgL.Tick()){
      l = true;
    }
    if(cgR.Tick()){
      r = true;
    }
    Serial.print(".");
    delay(1);
  }while(err!=0 || !l || !r);
  Serial.println();

  cgL.SetRunMode(2);
  cgR.SetRunMode(2);
  cgL.Enable();
  cgR.Enable();
  //cgL.SendFloat(/*Cybergear_parameter_Limit_Current*/0x7018, 10.0f);
  //cgL.SendFloat(/*Cybergear_parameter_Limit_Current*/0x7018, 10.0f);
  //cgL.SendFloat(/*ADDR_SPEED_KP*/0x2014, 2.0f);
  //cgR.SendFloat(/*ADDR_SPEED_KP*/0x2014, 2.0f);
  //cgL.SendFloat(/*ADDR_SPEED_KI*/0x2015, 0.0f);
  //cgR.SendFloat(/*ADDR_SPEED_KI*/0x2015, 0.0f);
  Serial.println("Ready");
}
uint8_t iseCnt=0;
float iseSum=0;
float stab = 1;

void iseFunc(float err){
  iseSum += err*err;
  iseCnt++;
  if(iseCnt>=100){
    stab = iseSum/iseCnt;
    SendNMsg('E', stab);
    iseCnt /= 2;
    iseSum /= 2;
    VelocityPID.I = VelI*_constrain(stab, 0.1, 1.0);
    SendNMsg('I', VelocityPID.integral_prev);
  }
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
    float a = bno.GetRoll() / 916.73247220931713402877047702568;
    //float b = bno.GetGyroY() / 916.73247220931713402877047702568;
    float v;
    if(abs(a)>0.75){
      VelocityPID.reset();
      AnglePID.reset();
      v = 0;
      turn = 0;
      stab = 1;
    } else {
      float vErr = (cgL.velocity - cgR.velocity) - tVelocity;
      float aExp = VelocityPID(vErr);
      v = AnglePID(a + aExp);

      iseFunc(vErr*stabK);
    }
    if((cgL.GetMotorStatus()&0xC0) == 0x80){
      cgL.Command(v+turn);
    }    
    if((cgR.GetMotorStatus()&0xC0) == 0x80){
      cgR.Command(turn-v);
    }
    //Serial.print(a);Serial.print(",");
    //Serial.print(v);//Serial.print(",");
    //Serial.print((cgL.torque - cgR.torque));
    //Serial.println();
  } else if(cgL.Tick()){
  } else if(cgR.Tick()){
  } else {
    Terminal();
  }
}
