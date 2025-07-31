//CPU - STM32F051C8Tx

#include "Wire.h"
#define I2C_DEV_ADDR 0x17
#define DELAY_MS 10
#define OFF_CNT (1000/DELAY_MS)

uint8_t offCnt = 0;
uint8_t sndBuff[6];

void setup() {
  pinMode(PA5, INPUT_PULLUP);          // Button 5
  pinMode(PA6, INPUT_PULLUP);          // Button 3
  pinMode(PA7, INPUT_PULLUP);          // Button Down
  pinMode(PA8, OUTPUT);                // Beep
  pinMode(PA11, OUTPUT);               // Power ON
  pinMode(PA12, INPUT);                // Button ON
  pinMode(PB0, INPUT_PULLUP);          // Button UP
  pinMode(PB1, OUTPUT);                // LCD Backlight
  Wire.setSCL(PB8);                    // SCL
  Wire.setSDA(PB9);                    // SDA
  pinMode(PB10, INPUT_PULLUP);         // Button Fire
  pinMode(PC13, INPUT_PULLUP);         // Button Back
  pinMode(PC15, INPUT_PULLUP);         // Button 6
  pinMode(PF1, INPUT_PULLUP);          // Button 4
  pinMode(PF7, INPUT_PULLUP);          // Button Ok

  Wire.begin();

  digitalWrite(PA11, HIGH);
  digitalWrite(PB1, LOW);
  digitalWrite(PA8, HIGH);
  delay(10);
  digitalWrite(PA8, LOW);
  while(!digitalRead(PA12)){
    delay(10);
  }
}

void loop() {
  uint32_t tmp;
  uint8_t b1 = digitalRead(PA12)?0:0x10;

  if(b1){
    offCnt++;
    if(offCnt>OFF_CNT){
      digitalWrite(PA8, offCnt == OFF_CNT+1?HIGH:LOW);  // Beep
      digitalWrite(PB1, HIGH);
      if(offCnt>250){
        offCnt = OFF_CNT+1;
      }
    }
  } else if(offCnt>0){
    offCnt--;
    if(offCnt>=OFF_CNT){
      digitalWrite(PA11, LOW);  // Power OFF
    }
  }
  tmp = analogRead(PA3);  // V_USB
  b1 = b1 | (digitalRead(PB10)?0:0x20) | (tmp>240?0x80:0); 

  tmp = analogRead(PA0);  // Vcc [250 - 284]
  sndBuff[0] = (uint8_t)(tmp>>4);
  sndBuff[1] = (tmp&0x0F) | b1;  // ON, Fire, , USB

  tmp = analogRead(PA1);  // X
  sndBuff[2] = (uint8_t)(tmp>>4);
  sndBuff[3] = (tmp&0x0F) | (digitalRead(PB0)?0:0x10) | (digitalRead(PA7)?0:0x20) | (digitalRead(PF7)?0:0x40) | (digitalRead(PC13)?0:0x80); // Up, Down, Ok, Back

  tmp = analogRead(PA2);  // Y
  sndBuff[4] = (uint8_t)(tmp>>4);
  sndBuff[5] = (tmp&0x0F) | (digitalRead(PA6)?0:0x10) | (digitalRead(PF1)?0:0x20) | (digitalRead(PA5)?0:0x40) | (digitalRead(PC15)?0:0x80); // B3, B4, B5, B6

  Wire.beginTransmission(I2C_DEV_ADDR);
  for(uint8_t i = 0; i<6;i++){
    Wire.write(sndBuff[i]);
  }
  Wire.endTransmission(true);

  delay(DELAY_MS);
}
