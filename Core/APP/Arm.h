#ifndef ARM_H
#define ARM_H
#include <stdint.h>

/* 底层：直接设置关节角度（带步进插值） */
void Arm_Init(void);
void Arm_SetAngle(float j1, float j2, float j3);

/* 便捷动作 */
void Arm_PickTop(void);
void Arm_PickBottom(void);
void Arm_Store(void);
void Arm_Suck(void);

/* 主任务：设置目标角度并发送指令（main 的 while(1) 里调用） */
void Arm_Task(uint8_t EN, uint8_t cup, float j1, float j2, float j3);

#endif
