#include "Wire.h"
#define I2C_DEV_ADDR 0x17

void setup() {
  pinMode(PA12, INPUT);         // Button ON
  pinMode(PA11, OUTPUT);        // Power ON
  pinMode(PA5, INPUT_PULLUP);          // Button 5
  pinMode(PA6, INPUT_PULLUP);          // Button 3
  pinMode(PA7, INPUT_PULLUP);          // Button Down
  pinMode(PB0, INPUT_PULLUP);          // Button UP
  pinMode(PB10, INPUT_PULLUP);         // Button Fire
  pinMode(PC13, INPUT_PULLUP);         // Button Back
  pinMode(PC15, INPUT_PULLUP);         // Button 6
  pinMode(PF1, INPUT_PULLUP);          // Button 4
  pinMode(PF7, INPUT_PULLUP);          // Button Ok

  Wire.setSCL(PB8);
  Wire.setSDA(PB9);


  delay(10);
  digitalWrite(PA11, HIGH);
  Wire.begin();
  while(!digitalRead(PA12)){
    delay(10);
  }
}

uint8_t i = 0;
uint8_t state = 0;

uint8_t Send(uint8_t cmd, uint32_t val){
  Wire.beginTransmission(I2C_DEV_ADDR);
  Wire.write(cmd);
  Wire.write((uint8_t)(val>>24));
  Wire.write((uint8_t)(val>>16));
  Wire.write((uint8_t)(val>>8));
  Wire.write((uint8_t)val);
  return Wire.endTransmission(true);
}
void loop() {
  delay(20);
  uint32_t t_u32;
  uint8_t t_u8;

  if(!digitalRead(PA12)){
    i++;
  } else if(i>0){
    i--;
    if(i>50){
      digitalWrite(PA11, LOW);
    }
  }

  switch(state++){
    case 0:
    case 1:
    case 2:
    case 3:
      //Send('V', analogRead(PA3));
      break;
    case 4:
      t_u32 = 0;
      t_u32 |= digitalRead(PA5)?0:0x01;
      t_u32 |= digitalRead(PA6)?0:0x02;
      t_u32 |= digitalRead(PA7)?0:0x04;
      t_u32 |= digitalRead(PB0)?0:0x08;
      t_u32 |= digitalRead(PB10)?0:0x10;
      t_u32 |= digitalRead(PC13)?0:0x20;
      t_u32 |= digitalRead(PC15)?0:0x40;
      t_u32 |= digitalRead(PF7)?0:0x80;
      t_u32 |= digitalRead(PF1)?0:0x100;
      Send('I', t_u32);
      break;
    default:
      state = 0;
      break;
  }
  /*
  // Read 16 bytes from the slave
  uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);
  
  Serial.printf("requestFrom: %u\n", bytesReceived);
  if ((bool)bytesReceived) {  //If received more than zero bytes
    uint8_t temp[bytesReceived];
    Wire.readBytes(temp, bytesReceived);
    log_print_buf(temp, bytesReceived);
  }
  */
}
