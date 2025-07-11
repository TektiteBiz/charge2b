#include "peripheral.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void LEDWrite(bool LED1, float r, float g, float b) {
  TIM_HandleTypeDef* htim = LED1 ? &htim1 : &htim3;
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(r * 65535.0f));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(g * 65535.0f));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(b * 65535.0f));
}