#include "Arm.h"
#include "dm_motor_ctrl.h"
#include "Serial.h"

#define J1  Motor1
#define J2  Motor2
#define J3  Motor3
#define ARM_STEP  0.17f//步长

void Arm_Init(void){

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET);

    dm_motor_init(J1, &hfdcan2, 0x01, mit_mode);
    dm_motor_init(J2, &hfdcan2, 0x02, mit_mode);
    dm_motor_init(J3, &hfdcan2, 0x03, mit_mode);

    mit_ctrl(motor[J1].hcan, &motor[J1], motor[J1].id,motor[J1].para.pos,0,100,15,0);
    mit_ctrl(motor[J2].hcan, &motor[J2], motor[J2].id,motor[J2].para.pos,0,90,13,0);
    mit_ctrl(motor[J3].hcan, &motor[J3], motor[J3].id,motor[J3].para.pos,0,9,2,0);

}


void Arm_SetAngle(float j1, float j2, float j3){
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 5) return;
    last_tick = now;

    float c1 = motor[J1].para.pos;
    float c2 = motor[J2].para.pos;
    float c3 = motor[J3].para.pos;
    float d1 = j1 - c1;
    float d2 = j2 - c2;
    float d3 = j3 - c3;

    if (d1 > ARM_STEP)       c1 += ARM_STEP;
    else if (d1 < -ARM_STEP) c1 -= ARM_STEP;
    else                     c1  = j1;

    if (d2 > ARM_STEP)       c2 += ARM_STEP;
    else if (d2 < -ARM_STEP) c2 -= ARM_STEP;
    else                     c2  = j2;

    if (d3 > ARM_STEP)       c3 += ARM_STEP;
    else if (d3 < -ARM_STEP) c3 -= ARM_STEP;
    else                     c3  = j3;

    mit_ctrl(motor[J1].hcan, &motor[J1], motor[J1].id, c1, 0, 180, 15, 0);
    mit_ctrl(motor[J2].hcan, &motor[J2], motor[J2].id, c2, 0, 100, 15 , 0);
    mit_ctrl(motor[J3].hcan, &motor[J3], motor[J3].id, c3, 0, 30, 9, 0);
}

void Arm_PickTop(void){
    Arm_SetAngle(0, 0, 0);
}

void Arm_PickBotton(void){
    Arm_SetAngle(0, 0, 0);
}

void Arm_Task(uint8_t EN, uint8_t cup, float j1, float j2, float j3){
    if(EN) {
        Arm_SetAngle(j1, j2, j3);
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, cup);
    Serial_Printf("%f,%f,%f,%f,%f,%f\n",j1, motor[J1].para.pos, j2, motor[J2].para.pos, j3, motor[J3].para.pos);
}