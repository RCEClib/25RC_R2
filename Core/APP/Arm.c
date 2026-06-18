#include "Arm.h"
#include "dm_motor_ctrl.h"

#define J1  Motor1
#define J2  Motor2
#define J3  Motor3

void Arm_Init(void){
    dm_motor_init(J1, &hfdcan2, 0x01, mit_mode);
    dm_motor_init(J2, &hfdcan2, 0x02, mit_mode);
    dm_motor_init(J3, &hfdcan2, 0x03, mit_mode);
}

void Arm_SetAngle(float j1, float j2, float j3){
    mit_ctrl(motor[J1].hcan, &motor[J1], motor[J1].id,j1,0,9,0,0);
    mit_ctrl(motor[J2].hcan, &motor[J2], motor[J2].id,j2,0,9,0,0);
    mit_ctrl(motor[J3].hcan, &motor[J3], motor[J3].id,j3,0,9,0,0);
}

void Arm_Task(float j1, float j2, float j3){
    static uint8_t inited = 0;

    if (!inited){
        Arm_Init();
        inited = 1;
    }

    Arm_SetAngle(j1, j2, j3);
    //HAL_Delay(10);
}
