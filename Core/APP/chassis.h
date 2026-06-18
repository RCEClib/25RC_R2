#ifndef __CHASSIS_RUDDER_H__
#define __CHASSIS_RUDDER_H__

#include <stdint.h>
#include "pid.h"

// 调试变量（便于上位机监控）
extern float target_speed;      // 轮速PID目标值（RPM）
extern float actual_speed;      // 轮速PID反馈值（RPM）
extern float left_current_out;  // 左轮输出电流
extern float right_current_out; // 右轮输出电流

typedef enum {
    STOP_MODE = 0,        // 停止模式
    NORMAL_MODE = 1,      // 正常模式
} Chassis_Mode;

// 底盘控制结构体
typedef struct {
    Chassis_Mode Mode;          // 底盘模式

    float Target_Vx;             // 目标前进速度 (m/s)
    float Target_Omega;          // 目标角速度 (rad/s)

    float target_left_rpm;       // 左轮目标转速 (RPM)
    float target_right_rpm;      // 右轮目标转速 (RPM)

    PID_Controller wheel_pid[4]; // 四个轮子的速度PID（位置式）
    PID_Controller taisheng_pid[2];  // 两个抬升轮的速度PID（位置式）
    int16_t wheel_currents[4];   // 四个轮子的目标电流
    int16_t taisheng_currents[2];   // 两个抬升轮的目标电流

    int8_t wheel_direction_calibration[4];  // 电机方向校准 (1或-1)
    int8_t taisheng_direction_calibration[2];  // 电机方向校准 (1或-1)  抬升轮
} Chassis_t;

// 函数声明
void Chassis_Init(Chassis_t *chassis);
void diff_solve(float vx, float omega, float *out_left_rpm, float *out_right_rpm);
void Chassis_Control(Chassis_t *chassis);
void Chassis_Task(Chassis_t *Chassis, Chassis_Mode mode, float vx, float vy, float vw, int8_t valve);

#endif
