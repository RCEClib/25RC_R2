#include"stm32h7xx_hal.h"
#include "tim.h"

void tuigan_Task(uint8_t mode)
{
  if (mode == 0) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET );
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET );
  }
  else if (mode == 1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET );
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET );
  }
  else if (mode == 2) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET );
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET );
  }
}

void Set_Angle(float angle) {
  /* 限制角度范围 */
  if(angle > 270.0f) angle = 270.0f;
  if(angle < 0.0f)   angle = 0.0f;

  /* angle -> CCR */
  uint32_t ccr = (uint32_t)(500 + (angle / 270.0f) * 2000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr);
}
