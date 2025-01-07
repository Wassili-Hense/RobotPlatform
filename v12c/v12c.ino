#include "twai.h"
#include "xiaomi_cybergear_defs.h"

uint8_t CYBERGEAR_CAN_ID = 0x7F;
uint8_t MASTER_CAN_ID = 0x00;

TWAI twai = TWAI(/*RX_PIN=*/4, /*TX_PIN=*/5);

float _uint_to_float(uint16_t x, float x_min, float x_max){
    uint16_t type_max = 0xFFFF;
    float span = x_max - x_min;
    return (float) x / type_max * span + x_min;
}

int status_cb(uint32_t identifier, uint8_t length, uint8_t *data){
    float position = _uint_to_float((uint16_t)data[1] | data[0] << 8, POS_MIN, POS_MAX);
    float speed = _uint_to_float((uint16_t)data[3] | data[2] << 8, V_MIN, V_MAX);
    float torque = _uint_to_float((uint16_t)data[5] | data[4] << 8, T_MIN, T_MAX);
    float temperature = (float)(data[7] | data[6] << 8)/10.0f;
    Serial.printf("POS:%f V:%f T:%f temp:%f\n", position, speed, torque, temperature);

  return 0;
}

void setup() {
  Serial.begin(115200);
  twai.Subscribe((CMD_REQUEST<<24) |(((uint32_t)CYBERGEAR_CAN_ID)<<8), 0x1F00FF00, status_cb);
}

void send_can_package(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t* data){
    uint32_t id = cmd_id << 24 | option << 8 | can_id;
    twai.Send(id, len, data);
}
void request_status() {
    uint8_t data[8] = {0x00};
    send_can_package(CYBERGEAR_CAN_ID, CMD_GET_STATUS, MASTER_CAN_ID, 8, data);
}  

void loop() {
  int err = twai.Tick();
  if(err!=0){
    Serial.println();
    Serial.print("$");
    Serial.println(err);
    delay(1000);
    return;
  }
  delay(1000);
  request_status();
}
