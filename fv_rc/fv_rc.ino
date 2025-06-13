/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

uint8_t tgtAddr[] = {0xCC, 0x7B, 0x5C, 0xA7, 0x55, 0x44};

struct nMsg {
  char cmd;
  float val;
};

nMsg myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    Serial.println(" - failed");
  }
}
void Command(char cmd, float val){
  digitalWrite(LED_BUILTIN, HIGH);
  myData.cmd = cmd;
  myData.val = val;
  Serial.print(cmd);Serial.print("$");Serial.print(val);
  esp_err_t result = esp_now_send(tgtAddr, (uint8_t *) &myData, sizeof(myData));
  if(result != ESP_OK){
    Serial.println(" - not connected");
  }
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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(len==2 && incomingData[0]==0x11){
    Serial.print(" - ");Serial.println((int8_t)incomingData[1]);
  }
}
/*
#include <esp_wifi.h>
void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}*/
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  } else {
    Serial.println("Error initializing ESP-NOW");
  }
  /*
  WiFi.STA.begin();
  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
  */
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

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
    return;
  }
}
 
void loop() {
  Terminal();

}