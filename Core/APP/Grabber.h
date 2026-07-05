#ifndef _GRABBER_H
#define _GRABBER_H

#include "main.h"

// ==========================================================
// 舵机角度定义
// ==========================================================
#define GRABBER_ANGLE_HORIZONTAL  105.0f   // 水平位置（初始位置）
#define GRABBER_ANGLE_VERTICAL    19.5f    // 垂直位置（收起位置）
#define GRABBER_ANGLE_MIN         0.0f     // 最小角度
#define GRABBER_ANGLE_MAX         270.0f   // 最大角度
#define GRABBER_ANGLE_TOLERANCE   0.1f     // 到达判定容差

// ==========================================================
// 函数声明
// ==========================================================
void Grabber_Init(void);
void Set_Angle270(float angle);
void sevo270(float start_angle, float end_angle, float step);


void Grabber_Task(uint8_t se_pressed, float step, uint16_t delay_ms);

// 阻塞式运动到目标角度（完成后自动发送ACK）
void Grabber_Move_To_Angle(float target_angle, float step, uint16_t delay_ms);

// 获取当前角度
float Grabber_Get_Current_Angle(void);

#endif //_GRABBER_H
