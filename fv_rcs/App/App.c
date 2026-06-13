#include "App.h"

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include "adc_inputs.h"
#include "buttons.h"
#include "st7735.h"
#include "Timers.h"

#define OFF_CNT 1000
static uint16_t blCnt = 5000;

void PowerOff() {
	static uint16_t offCnt = 0;

	if (ADC_V < 950) {  // Vbat < 3.2V
		if (offCnt >= OFF_CNT) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // Power OFF
		} else {
			offCnt++;
			if (offCnt == OFF_CNT / 2) {
				Tone(500, 60);
			}
		}
	} else if (Buttons_Get(0)) {
		offCnt++;
		if (offCnt > OFF_CNT) {
			if (offCnt == OFF_CNT + 2) {
				Tone(500, 10);
			}
			if (offCnt * 3 > OFF_CNT * 4) {
				offCnt = OFF_CNT + 1;
			}
		} else {
			blCnt = 15000;
		}
	} else if (offCnt > 0) {
		if (offCnt >= OFF_CNT) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); // Power OFF
		} else {
			offCnt--;
		}
	}
}
void BackLight() {
	if (blCnt > 0) {
		SetBackLight(blCnt < 1024 ? (blCnt >> 3) : 127);
		blCnt--;
	}
}

void App_Init(void) {
	AdcInputs_Init();
	Buttons_Init();
	ST7735_Init();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); // Power ON
	SetBackLight(63);
	Tone(757, 45); // E6
	HAL_Delay(65);

	ST7735_Clear(ST7735_BLACK);

	while (Buttons_Get(0)) {
		ST7735_Process();
		__WFI();
	}

	Tone(636, 60); // G6
	HAL_Delay(90);
	Tone(476, 90); // C7
}

void App_Run(void) {
	ST7735_Process();
	__WFI();
}

void HAL_SYSTICK_Callback(void) {
	Buttons_Tick1ms();
	PowerOff();
	BackLight();
	Tone_Process();
}
