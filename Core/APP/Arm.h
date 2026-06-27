#ifndef ARM_H
#define ARM_H
#include <stdint.h>

void Arm_Init(void);
void Arm_SetAngle(float j1, float j2, float j3);
void Arm_SetStiffness(float kp, float kd);
void Arm_Solve(void);
void Arm_GetAngle(float *j1, float *j2, float *j3);

/* 主任务：设置目标角度并发送指令（main 的 while(1) 里调用） */
void Arm_Task(uint8_t EN, uint8_t cup, float j1, float j2, float j3);

#endif