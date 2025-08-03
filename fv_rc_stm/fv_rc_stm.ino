//CPU - STM32F051C8Tx

#include "TInput.hpp"

#define DELAY_MS 2
#define OFF_CNT (1000/DELAY_MS)

#define I2C_DEV_ADDR 0x17

//uint8_t sndBuff[6];
static uint16_t blCnt = OFF_CNT*2;

TInput inputs[] = {
  TInput(true, PA0,  'V'),     // Vcc [1000 - 1136]
  TInput(true, PA1,  'X'),
  TInput(true, PA2,  'Y'),
  TInput(true, PA3,  'U'),
  TInput(false, PA5,   5),     // Button 5
  TInput(false, PA6,   3),     // Button 3
  TInput(false, PA7,   8),     // Button Down
  TInput(false, PA12,  1),     // Button ON
  TInput(false, PB0,   7),     // Button UP
  TInput(false, PB10,  2),     // Button Fire
  TInput(false, PC13, 10),     // Button Back
  TInput(false, PC15,  6),     // Button 6
  TInput(false, PF1,   4),     // Button 4
  TInput(false, PF7,   9)      // Button Ok
};

#define INPUTS_CNT (sizeof(inputs)/sizeof(TInput))
void Send(uint8_t cmd, uint16_t value){
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(cmd);
  Wire.write((uint8_t)(value>>8));
  Wire.write((uint8_t)value);
  Wire.endTransmission(true);
}

void setup() {
  analogReadResolution(12);

  pinMode(PA8, OUTPUT);        // Beep
  pinMode(PA11, OUTPUT);       // Power ON
  pinMode(PB1, OUTPUT);        // LCD Backlight
  Wire.setSCL(PB8);            // SCL
  Wire.setSDA(PB9);            // SDA
  TInput::SendFunc = &Send;
  for(uint8_t i=0; i<INPUTS_CNT; i++){
    inputs[i].Init();
  }

  digitalWrite(PB1, HIGH);     // LCD Backlight OFF
  Wire.begin();

  digitalWrite(PA11, HIGH);    // Power ON
  analogWrite(PB_1_ALT2, 192); // LCD Backlight 25%
  tone(PA8, 600, 25);
  
  while(!digitalRead(PA12)){
    delay(10);
  }
}

void PowerOff(){
  static uint16_t offCnt = 0;

  if(!digitalRead(PA12)){
    offCnt++;
    if(offCnt>OFF_CNT){
      if(offCnt == OFF_CNT+2){
        tone(PA8, 1500, 10);
        digitalWrite(PB1, HIGH);
      }
      if(offCnt*3>OFF_CNT*4){
        offCnt = OFF_CNT+1;
      }
    } else {
      blCnt = OFF_CNT;
    }
  } else if(offCnt>0){
    offCnt--;
    if(offCnt>=OFF_CNT){
      digitalWrite(PA11, LOW);  // Power OFF
    }
  }
}

void BackLight(){
  if(blCnt>0){
    analogWrite(PB_1_ALT2, blCnt<256?(256-blCnt):0);
    blCnt--;
  }
}

void loop() {
  static uint8_t valIdx = 0;
  for(uint8_t i=0; i<INPUTS_CNT; i++){
    inputs[i].Tick();
  }
  
  
  while(true){
    if(inputs[valIdx].Send()) break;
    if(valIdx>=INPUTS_CNT-1){
      valIdx=0;
      break;
    } else {
      valIdx++;
    }
  }
  PowerOff();
  BackLight();
  delay(DELAY_MS);
}
