#include "xiaomi_cybergear.h"

TWAI twai = TWAI(/*RX_PIN=*/4, /*TX_PIN=*/5);
Cybergear cg = Cybergear(&twai, 0x7F);

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
}
