#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "TWAI.h"
#include "driver/twai.h"
//https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html

#include <HardwareSerial.h>

TWAI::TWAI(uint8_t rx_pin, uint8_t tx_pin) : _rx_pin(rx_pin), _tx_pin(tx_pin), _cbHead(nullptr), _cbTail(nullptr) { }
TWAI::~TWAI(){
  if (twai_stop() != ESP_OK) return;
  twai_driver_uninstall();
}
int8_t TWAI::Tick(){
  twai_status_info_t twai_status;
  if(twai_get_status_info(&twai_status)!= ESP_OK || twai_status.state != TWAI_STATE_RUNNING){
    int err = Init();
    if(err!=0){
      return err;
    }  
    if(twai_get_status_info(&twai_status)!= ESP_OK) return -4;  // Failed to get twai status  
    if(twai_status.state != TWAI_STATE_RUNNING) return -5; // Twai not runing
    return 0;
  }
  
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, 0);
  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) return -6; // TWAI controller has become error passive
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) return -7; // Error has occurred on the bus
  if (alerts_triggered & TWAI_ALERT_TX_FAILED) return -8; // The Transmission failed
  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t msg;
    TWAI_Sub *cur;
    int8_t err;
    while (twai_receive(&msg, 0) == ESP_OK) {
      cur = _cbHead;
      while(cur){
        err=cur->Do(msg.identifier, msg.data_length_code, msg.data);
        if(err!=0) return err;
        cur = cur->next();
      }
    }
    //Serial.print("RC:");Serial.println(msg.identifier, HEX);
  } else if(twai_status.msgs_to_tx==0 && _mqHead){
    twai_message_t message;
    message.extd = 1; //enable extended frame format
    message.identifier = _mqHead->Identifier();
    message.data_length_code = _mqHead->Length();
    for (int i = 0; i < _mqHead->Length(); i++) {
      message.data[i] = _mqHead->Data()[i];
    }
    if(twai_transmit(&message, 0) == ESP_OK){
      if(!_mqHead->Next()){
        _mqTail = NULL;  
      }
      _mqHead = _mqHead->Next();
      //Serial.print("SC:");Serial.println(message.identifier, HEX);
    }    
  }

  return 0;
}
void TWAI::Subscribe(uint32_t value, uint32_t mask, std::function<int8_t(uint32_t identifier, uint8_t length, uint8_t *data)> cb){
  TWAI_Sub *h = new TWAI_Sub(value, mask, cb);
  if (!_cbTail) {
    _cbHead = h;
    _cbTail = h;
  } else {
    _cbTail->next(h);
    _cbTail = h;
  }
}
int8_t TWAI::Send(uint32_t identifier, uint8_t length, uint8_t *data){
  TWAI_Msg *m = new TWAI_Msg(identifier, length, data);
  if(!m) return -9;
  if (!_mqTail) {
    _mqHead = m;
    _mqTail = m;
  } else {
    _mqTail->Next(m);
    _mqTail = m;
  }
  //Serial.print("QC:");Serial.println(identifier, HEX);
  return 0;
}
int8_t TWAI::Init(){
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)_tx_pin, (gpio_num_t)_rx_pin, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if(twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) return -1;  // Failed to install driver

  //Start TWAI driver
  if (twai_start() != ESP_OK) return -2;  // Failed to start driver

  // Reconfigure alerts to detect TX alerts and Bus-Off errors
  //uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) return -3;  // Failed to reconfigure alerts
  return 0;
}

TWAI_Sub::TWAI_Sub(uint32_t value, uint32_t mask, std::function<int8_t(uint32_t identifier, uint8_t length, uint8_t *data)> cb){
  _value=value;
  _mask=mask;
  _cb=cb;
}

int8_t TWAI_Sub::Do(uint32_t identifier, uint8_t length, uint8_t *data){
  if(((identifier ^ _value)&_mask)!=0){
    return 0;
  }
  return _cb(identifier, length, data);
}

