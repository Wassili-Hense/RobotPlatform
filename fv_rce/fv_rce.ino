#include <Wire.h>

#define I2C_ADDR 20
#define SDA_PIN 21
#define SCL_PIN 22

// Тайминг
unsigned long lastPoll = 0;

// Текущее состояние
uint16_t adcX = 2048;
uint16_t adcY = 2048;
uint8_t usbConnected = 0;
uint8_t lcdBusy = 0;
uint8_t buttonIndex = 0;

// Для отслеживания изменений
uint16_t prevAdcX = 2048;
uint16_t prevAdcY = 2048;
uint8_t prevUsb = 255;
uint8_t prevButton = 255;

// Имена кнопок по индексам
const char* btnNames[] = {
  "", "", "", "ON", "Fire", "UP", "DOWN", "BACK", "OK",
  "LUP", "LDN", "RUP", "RDN"
};

bool serialActive = false;

// ---------------- LCD COMMAND SENDER ----------------
void lcdSend(uint8_t cmd, const uint8_t* data, uint8_t len) {
  if (lcdBusy) return;  // очередь занята — не шлём

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(cmd);
  for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
  Wire.endTransmission();
}

// Быстрые функции
void lcdFillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
  uint8_t buf[6] = {x, y, w, h, (uint8_t)(color & 0xFF), (uint8_t)(color >> 8)};
  lcdSend(0x13, buf, 6);
}

// ---------------- READ SLAVE ----------------
bool readSlave(uint8_t &index, uint16_t &value) {
  Wire.requestFrom(I2C_ADDR, 2);
  if (Wire.available() < 2) return false;

  uint8_t b0 = Wire.read();
  uint8_t b1 = Wire.read();

  index = b0 >> 4;
  value = ((b0 & 0x0F) << 8) | b1;
  return true;
}

// ---------------- PRINT SERIAL ----------------
void printState(uint8_t idx, uint8_t status, float nx, float ny, const char* btn) {
  if (!serialActive) return;

  Serial.print(idx);
  Serial.print(" ");
  Serial.print(status);
  Serial.print(" ");
  Serial.print(nx, 3);
  Serial.print(" ");
  Serial.print(ny, 3);
  Serial.print(" ");
  Serial.println(btn);
}

// ---------------- SETUP ----------------
void setup() {
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long now = millis();
  if (now - lastPoll < 20) return;
  lastPoll = now;

  uint8_t idx;
  uint16_t val;

  if (!readSlave(idx, val)) return;

  // Обработка статуса
  if (idx == 0) {
    lcdBusy = val & 0x01;
    usbConnected = (val >> 1) & 0x01;

    // USB логика
    if (usbConnected != prevUsb) {
      prevUsb = usbConnected;

      if (usbConnected) {
        if (!serialActive) {
          Serial.begin(115200);
          serialActive = true;
        }
        // Синий квадрат 5x5 в служебной области [155,0]
        lcdFillRect(155, 0, 5, 5, 0x001F);
      } else {
        if (serialActive) {
          Serial.end();
          serialActive = false;
        }
        // Чёрный квадрат
        lcdFillRect(155, 0, 5, 5, 0x0000);
      }
    }
  }

  // ADC X
  if (idx == 1) {
    adcX = val;
  }

  // ADC Y
  if (idx == 2) {
    adcY = val;
  }

  // Кнопки
  if (idx >= 3 && idx <= 12) {
    if (val > 0) buttonIndex = idx;
    else if (buttonIndex == idx) buttonIndex = 0;
  }

  // Проверка изменений
  bool changed = false;

  if (adcX != prevAdcX) { prevAdcX = adcX; changed = true; }
  if (adcY != prevAdcY) { prevAdcY = adcY; changed = true; }
  if (buttonIndex != prevButton) { prevButton = buttonIndex; changed = true; }

  if (changed) {
    float nx = (adcX - 2048) / 2048.0f;
    float ny = (adcY - 2048) / 2048.0f;

    printState(idx, (usbConnected << 1) | lcdBusy, nx, ny,
               buttonIndex ? btnNames[buttonIndex] : "");
  }
}
