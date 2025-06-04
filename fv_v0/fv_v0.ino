#include <Wire.h> //include wire.h for i2c communication
#include "bno055.hpp"

BNO055 bno(&Wire, false);

void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  Wire.begin(); //Start I2C communication as master
  Wire.setClock(400000);  //set I2C to fast mode at 400kHz

  int8_t err = bno.Initialize();
  //int8_t err = 1;
  if(err!=0){
    Serial.print("BNO055.Init - "); 
    Serial.println(err);
    while(1){
      delay(1000);
    }
  }
}

void loop() {
  bno.Read();
  Serial.print(bno.heading/16.0);
  Serial.print(", ");
  Serial.print(bno.roll/16.0);
  Serial.print(", ");
  Serial.println(bno.pitch/16.0);
  delay(100);
}
