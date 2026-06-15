#include "stm32h7xx_hal.h"
#include "Grapper.h"
#include "tim.h"

void Grapper_Init(void) {

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);//R2夹爪舵机用

    //继电器控制电磁阀夹爪
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

}


