#include "stm32h7xx_hal.h"
#include "Grabber.h"
#include "tim.h"
#include "ros_serial.h"
#include <math.h>        // 用于 fabsf

// ==========================================================
// 全局变量
// ==========================================================
float current_angle = 0.0f;  // 当前实际角度

// ==========================================================
// 初始化函数
// ==========================================================
void Grabber_Init(void) {
    // 启动 PWM 定时器通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // R2夹爪舵机用

    // 继电器控制电磁阀夹爪（初始关闭）
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

    // 初始化角度
    current_angle = GRABBER_ANGLE_HORIZONTAL;
    Set_Angle270(GRABBER_ANGLE_HORIZONTAL);  // 初始水平位置
}

// ==========================================================
// 角度 → PWM 脉宽转换
// ==========================================================
/**270度舵机
 * @brief  将角度转换为 PWM 比较值 (CCR)
 * @param  angle: 目标角度 (0.0 - 270.0)
 * @retval 无
 * @note   脉宽范围: 0.5ms ~ 2.5ms 对应 CCR: 500 ~ 2500
 */
void Set_Angle270(float angle) {
    /* 限制角度范围 */
    if (angle > GRABBER_ANGLE_MAX) angle = GRABBER_ANGLE_MAX;
    if (angle < GRABBER_ANGLE_MIN) angle = GRABBER_ANGLE_MIN;

    /* angle -> CCR: 500 + (angle/270) * 2000 */
    uint32_t ccr = (uint32_t)(500 + (angle / 270.0f) * 2000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr);
}

// ==========================================================
// 连续运动（开环，无反馈）
// ==========================================================
/**270度舵机连续运动
 * @param start_angle: 起始角度
 * @param end_angle:   结束角度
 * @param step:        每次步进角度
 * @note  注意：此函数会阻塞执行
 */
void sevo270(float start_angle, float end_angle, float step) {
    if (start_angle < end_angle) {
        // 正向运动
        for (float ang = start_angle; ang <= end_angle; ang += step) {
            Set_Angle270(ang);
        }
    } else {
        // 反向运动
        for (float ang = start_angle; ang >= end_angle; ang -= step) {
            Set_Angle270(ang);
        }
    }
    // 确保最终位置精确
    Set_Angle270(end_angle);
}

// ==========================================================
// 遥控器控制函数（原有，保留兼容）
// ==========================================================
/**
 * @brief 遥控器控制夹爪（每帧调用一次）
 * @param se_pressed: 1=水平(105°), 0=垂直(19.5°)
 * @param step: 步进角度
 * @param delay_ms: 每步延时(ms)
 * @note  非阻塞，每次只移动一步
 */
void Grabber_Task(uint8_t se_pressed, float step, uint16_t delay_ms) {
    if (se_pressed) {
        // 水平位置 105°
        if (current_angle < GRABBER_ANGLE_HORIZONTAL) {
            current_angle += step;
            if (current_angle > GRABBER_ANGLE_HORIZONTAL) {
                current_angle = GRABBER_ANGLE_HORIZONTAL;
            }
            Set_Angle270(current_angle);
            HAL_Delay(delay_ms);  // 仅在运动时延迟
        }
    } else {
        // 垂直位置 19.5°
        if (current_angle > GRABBER_ANGLE_VERTICAL) {
            current_angle -= step;
            if (current_angle < GRABBER_ANGLE_VERTICAL) {
                current_angle = GRABBER_ANGLE_VERTICAL;
            }
            Set_Angle270(current_angle);
            HAL_Delay(delay_ms);  // 仅在运动时延迟
        }
    }
}

// ==========================================================
// ：阻塞式运动到目标角度（带ACK反馈）
// ==========================================================
/**
 * @brief 舵机平滑移动到目标角度（使用while循环，阻塞式）
 * @param target_angle: 目标角度
 * @param step: 每次步进角度（越小越平滑，但耗时越长）
 * @param delay_ms: 每一步的延时（ms），控制运动速度
 * @note  函数会阻塞直到到达目标角度，然后自动发送ACK
 *        调用前请确保 current_angle 已初始化
 */
void Grabber_Move_To_Angle(float target_angle, float step, uint16_t delay_ms) {
    // ===== 1. 限制目标角度范围 =====
    if (target_angle > GRABBER_ANGLE_MAX) {
        target_angle = GRABBER_ANGLE_MAX;
    }
    if (target_angle < GRABBER_ANGLE_MIN) {
        target_angle = GRABBER_ANGLE_MIN;
    }

    // ===== 检查是否已经在目标角度 =====
    // if (fabsf(current_angle - target_angle) < GRABBER_ANGLE_TOLERANCE) {
    //     // 已经在目标位置，直接发送ACK并返回
    //     ROS_Send_ACK();
    //     return;
    // }

    // ===== 判断运动方向 =====
    if (target_angle > current_angle) {
        // ---- （角度增大） ----
        while (current_angle < target_angle - GRABBER_ANGLE_TOLERANCE) {
            current_angle += step;
            if (current_angle > target_angle) {
                current_angle = target_angle;
            }
            Set_Angle270(current_angle);
            HAL_Delay(delay_ms);
        }
    } else {
        // ---- 角度减小 ----
        while (current_angle > target_angle + GRABBER_ANGLE_TOLERANCE) {
            current_angle -= step;
            if (current_angle < target_angle) {
                current_angle = target_angle;
            }
            Set_Angle270(current_angle);
            HAL_Delay(delay_ms);
        }
    }

    // =====  精确到达目标角度 =====
    current_angle = target_angle;
    Set_Angle270(current_angle);

    // =====  发送ACK确认完成 =====
   // ROS_Send_ACK();
}

// ==========================================================
// 获取当前角度
// ==========================================================
float Grabber_Get_Current_Angle(void) {
    return current_angle;
}