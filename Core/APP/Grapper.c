#include "stm32h7xx_hal.h"
#include "Grapper.h"
#include "tim.h"

void Grapper_Init(void) {

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);//R2夹爪舵机用

    //继电器控制电磁阀夹爪
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    Set_Angle270(25.0f);//初始水平位置
}



// /**180度舵机
//  * @brief  将角度转换为 PWM 比较值 (CCR)
//  * @param  angle: 目标角度 (0.0 - 270.0)
//  * @retval 对应的 CCR 值
//  */
// void Set_Angle180(float angle)
// {
//     if(angle > 180.0f) angle = 180.0f;
//     if(angle < 0.0f)   angle = 0.0f;
//     uint32_t ccr = (uint32_t)(500 + (angle / 180.0f) * 2000);
//     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr);
// }



/**270度舵机
 * @brief  将角度转换为 PWM 比较值 (CCR)
 * @param  angle: 目标角度 (0.0 - 270.0)
 * @retval 对应的 CCR 值
 */
void Set_Angle270(float angle) {
    /* 限制角度范围 */
    if(angle > 270.0f) angle = 270.0f;
    if(angle < 0.0f)   angle = 0.0f;

    /* angle -> CCR */
    uint32_t ccr = (uint32_t)(500 + (angle / 270.0f) * 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr);
}


//180度舵机
//start_angle: 起始角度
//end_angle: 结束角度
//step: 步长
// void sevo180(float start_angle, float end_angle, float step) {
//
//     for ( float ang = start_angle ; ang <= end_angle; ang += step) {
//         Set_Angle180(ang);
//     }
// }




//270度舵机
//start_angle: 起始角度
//end_angle: 结束角度
//step: 步长
void sevo270(float start_angle, float end_angle, float step) {


    for ( float ang = start_angle ; ang <= end_angle; ang += step) {
        Set_Angle270(ang);
    }
}
