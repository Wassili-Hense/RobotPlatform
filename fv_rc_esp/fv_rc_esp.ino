#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"

// ESP_NOW
uint8_t tgtAddr[] = {0xCC, 0x7B, 0x5C, 0xA7, 0x55, 0x44};

struct nMsg {
  char cmd;
  float val;
};

nMsg myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //if(status == ESP_NOW_SEND_SUCCESS){
  //  digitalWrite(LED_BUILTIN, LOW);
  //} else {
  //  Serial.println("!:-1");
  //}
}
void Command(char cmd, float val){
  //digitalWrite(LED_BUILTIN, HIGH);
  myData.cmd = cmd;
  myData.val = val;
  esp_err_t result = esp_now_send(tgtAddr, (uint8_t *) &myData, sizeof(myData));
  //Serial.print(result == ESP_OK?cmd:'!');Serial.print(":");Serial.println(val, 8);
}

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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  /*if(len==2 && incomingData[0]==0x11){
    if(incomingData[1]!=0){
      Serial.print("!:");Serial.println((int8_t)incomingData[1]);
    }
  } else */if(len==sizeof(myData) && sOpen){
    nMsg msg;
    memcpy(&msg, incomingData, sizeof(nMsg));
    Serial.print(msg.cmd);Serial.print(':');Serial.println(msg.val, 8);
  }
}


void setup() {
  Serial.end();
  //Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  } else {
    Serial.println("Error initializing ESP-NOW");
  }

  // Init ESP-NOW
  if (esp_now_init() = ESP_OK) {
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Transmitted packet
    esp_now_register_send_cb(OnDataSent);
  
    // Register peer
    memcpy(peerInfo.peer_addr, tgtAddr, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
  
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
    }
  } else {
    Serial.println("Error initializing ESP-NOW");
  }

  Wire.onReceive(onJReceive);
  //Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
}

float norm(uint16_t val){
  float r = val/2048.0;
  if(r<0) return 0;
  if(r>1) return 1;
  return r;
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
      uint16_t val = (((uint16_t)jrBuff[1])<<8) | jrBuff[2];
      if(sOpen){
        Serial.println(val);
      }
      switch(jrBuff[0]){
      case 'U':
        usbOn = val > 3*256;
        break;
      case 'X':
        Command('r', norm(val-2010));
        break;  
      case 'Y':
        Command('v', norm(2010-val));
        break;  
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
    } else if(sOpen){
     Terminal(); 
    }
  }
  delay(1);
}