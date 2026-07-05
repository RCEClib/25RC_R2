#include "Arm.h"
#include "dm_motor_ctrl.h"
#include "Serial.h"
#include <math.h>
#include <stdlib.h>

#define J1  Motor1
#define J2  Motor2
#define J3  Motor3
#define ARM_STEP  0.13f   // 单步最大弧度（≈10°）

#define ARM_L1          0.60f    // 肩→肘（米）
#define ARM_L2          0.42f    // 肘→腕（米）
#define ARM_L3          0.1365f  // 腕→末端（米）

/* ==================== 关节限位（弧度） ==================== */
#define J1_MIN   0.0f
#define J1_MAX   2.38f
#define J2_MIN  -5.5f
#define J2_MAX   0.1f
#define J3_MIN  -1.56f
#define J3_MAX   1.95f

/* ==================== 初始化 ==================== */
void Arm_Init(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);

    dm_motor_init(J1, &hfdcan2, 0x01, mit_mode);
    dm_motor_init(J2, &hfdcan2, 0x02, mit_mode);
    dm_motor_init(J3, &hfdcan2, 0x03, mit_mode);

    mit_ctrl(motor[J1].hcan, &motor[J1], motor[J1].id, motor[J1].para.pos, 0, 100, 15, 0);
    mit_ctrl(motor[J2].hcan, &motor[J2], motor[J2].id, motor[J2].para.pos, 0, 90, 13, 0);
    mit_ctrl(motor[J3].hcan, &motor[J3], motor[J3].id, motor[J3].para.pos, 0, 9, 2, 0);

}

void Arm_SetAngle(float j1, float j2, float j3)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 5) return;
    last_tick = now;

    /* 步进限幅：每 5ms 最多移动 ARM_STEP，慢走防抖 */
    float c1 = motor[J1].para.pos;
    float c2 = motor[J2].para.pos;
    float c3 = motor[J3].para.pos;

    float d1 = j1 - c1;
    float d2 = j2 - c2;
    float d3 = j3 - c3;

    /* 步进 clamp：每轴最多向目标靠近 ARM_STEP，绝不过冲 */
    c1 += fmaxf(-ARM_STEP, fminf(ARM_STEP, d1));
    c2 += fmaxf(-ARM_STEP, fminf(ARM_STEP, d2));
    c3 += fmaxf(-ARM_STEP, fminf(ARM_STEP, d3));

    /* 限位钳位 */
    c1 = fmaxf(J1_MIN, fminf(J1_MAX, c1));
    c2 = fmaxf(J2_MIN, fminf(J2_MAX, c2));
    c3 = fmaxf(J3_MIN, fminf(J3_MAX, c3));

    mit_ctrl(motor[J1].hcan, &motor[J1], motor[J1].id, c1, 0, 180, 15, 0);
    mit_ctrl(motor[J2].hcan, &motor[J2], motor[J2].id, c2, 0, 100, 15, 0);
    mit_ctrl(motor[J3].hcan, &motor[J3], motor[J3].id, c3, 0, 30, 9, 0);
}

//取方块顶部矿
void Arm_PickTop(void) {
    Arm_SetAngle(0, 0, 0);
}
//取方块底部矿
void Arm_PickBottom(void) {
    Arm_SetAngle(0, 0, 0);
}
//九宫格存矿
void Arm_Store(void) {
    Arm_SetAngle(0, 0, 0);
}
//吸着矿走
void Arm_Suck(void) {
    Arm_SetAngle(0, 0, 0);
}
//


void Arm_Task(uint8_t EN, uint8_t cup, float j1, float j2, float j3)
{
    if (EN) {
        Arm_SetAngle(j1, j2, j3);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, cup);
    // Serial_Printf("%f,%f,%f,%f,%f,%f\n",
    //               j1, motor[J1].para.pos,
    //               j2, motor[J2].para.pos,
    //               j3, motor[J3].para.pos);
}