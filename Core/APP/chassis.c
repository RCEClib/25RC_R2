#include "chassis.h"
#include "motor.h"
#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include "bsp_fdcan.h"

// ============================ 全局变量定义 ============================
float target_speed;        // 轮速PID目标值（RPM）
float actual_speed;        // 轮速PID反馈值（RPM）
float left_current_out;    // 左轮输出电流
float right_current_out;   // 右轮输出电流

// ============================ 宏定义 ============================

// 底盘几何参数（根据实际轮子和底盘尺寸调整）
#define WHEEL_RADIUS        0.10f          // 车轮半径（米）
#define WHEEL_BASE          0.51f           // 左右轮轮距（米），即矩阵中的 2*y

// 3508电机减速比（电机转19圈，轮子转1圈）
#define MOTOR_3508_GEAR_RATIO  19.0f

// 速度限幅
#define MAX_LINEAR_SPEED    2.5f             // 最大线速度 (m/s)
#define MAX_ANGULAR_SPEED   4.0f             // vy通道最大角速度 (rad/s)
#define MAX_GYRO_SPEED      6.0f             // vw通道小陀螺最大角速度 (rad/s)

// 安全保护：最大允许电流（根据电机和驱动器实际限制调整）
#define MAX_WHEEL_CURRENT   16384            // 3508电机最大电流
// 摇杆死区
#define JOYSTICK_DEADZONE   1.0f

// ============================ 辅助函数 ============================
/**
 * @brief 检查方向校准系数有效性，非法值返回1
 */
static int8_t CheckDirectionCalibration(int8_t val) {
    if (val == 1 || val == -1) return val;
    return 1;   // 默认为正向
}

/**
 * @brief 线速度(m/s) 转 电机转速(rpm)
 */
static float LinearToMotorRpm(float linear_speed) {
    float wheel_rps = linear_speed / (2.0f * M_PI * WHEEL_RADIUS);
    return wheel_rps * 60.0f * MOTOR_3508_GEAR_RATIO;
}

// ============================ 运动学解算（核心） ============================
/**
 * @brief 差速底盘运动学逆解（车体坐标系）
 * @param vx          车体前进速度（米/秒，向前为正）
 * @param omega       车体自转角速度（弧度/秒，逆时针为正）
 * @param out_left_rpm  输出左轮目标转速（RPM）
 * @param out_right_rpm 输出右轮目标转速（RPM）
 * @note 公式: vL = vx - y*ω, vR = vx + y*ω, 其中 y = 轮距/2
 */
void diff_solve(float vx, float omega, float *out_left_rpm, float *out_right_rpm) {
    // 限幅（防止超出电机能力）
    if (vx > MAX_LINEAR_SPEED) vx = MAX_LINEAR_SPEED;
    if (vx < -MAX_LINEAR_SPEED) vx = -MAX_LINEAR_SPEED;
    if (omega > MAX_ANGULAR_SPEED + MAX_GYRO_SPEED)
        omega = MAX_ANGULAR_SPEED + MAX_GYRO_SPEED;
    if (omega < -MAX_ANGULAR_SPEED - MAX_GYRO_SPEED)
        omega = -MAX_ANGULAR_SPEED - MAX_GYRO_SPEED;

    float half_track = WHEEL_BASE / 2.0f;   // 矩阵中的 y
    float v_left  = vx + half_track * omega;
    float v_right = vx - half_track * omega;

    *out_left_rpm  = LinearToMotorRpm(v_left);
    *out_right_rpm = LinearToMotorRpm(v_right);
}

// ============================ 初始化函数 ============================
void Chassis_Init(Chassis_t *chassis) {
    if (chassis == NULL) return;

    chassis->Target_Vx = 0.0f;
    chassis->Target_Omega = 0.0f;
    chassis->target_left_rpm = 0.0f;
    chassis->target_right_rpm = 0.0f;

    // 初始化轮向PID（位置式速度环，输入：RPM，输出：电流）
    for (int i = 0; i < 4; i++) {
        PID_Init(&chassis->wheel_pid[i],6.2f, 1.4f, 0.1f,MAX_WHEEL_CURRENT, 1000);
        chassis->wheel_currents[i] = 0;
    }

    // 电机方向校准（根据实际接线调整，1=正向，-1=反向）
    // 顺序：0-左前, 1-左后, 2-右后, 3-右前
    chassis->wheel_direction_calibration[0] = 1;   // 左前
    chassis->wheel_direction_calibration[1] = 1;   // 左后
    chassis->wheel_direction_calibration[2] = -1;   // 右后
    chassis->wheel_direction_calibration[3] = -1;   // 右前

    // 验证并修正校准系数
    for (int i = 0; i < 4; i++) {
        chassis->wheel_direction_calibration[i] = CheckDirectionCalibration(
            chassis->wheel_direction_calibration[i]);
    }

    // 发送零电流，使电机处于待机状态
    Motor_SendCurrent_Ex(&hfdcan1, MOTOR_3508_GROUP1, 0, 0, 0, 0);
}

// ============================ 差速控制（PID + 电流发送） ============================
void Chassis_Control(Chassis_t *chassis) {
    if (chassis == NULL) return;

    // ---------- 轮向控制（速度PID） ----------
    for (int i = 0; i < 4; i++) {
        // 获取当前电机实际转速（RPM），并乘以方向校准系数
        float actual_rpm = motor_feedback[MOTOR_3508_ID1_INDEX + i].speed *chassis->wheel_direction_calibration[i];

        // 目标转速：左轮(0,1)使用 target_left_rpm，右轮(2,3)使用 target_right_rpm
        float target_rpm = (i < 2) ? chassis->target_left_rpm : chassis->target_right_rpm;

        // 调试变量赋值（取左前轮和右前轮的值用于监控）
        if (i == 0) {
            target_speed = target_rpm;
            actual_speed = actual_rpm;
        }

        // PID计算目标电流
        int16_t raw_current = (int16_t)PID_Calculate(&chassis->wheel_pid[i],
                                                     target_rpm, actual_rpm);
        // 输出限幅保护（防止PID计算溢出或异常）
        if (raw_current > MAX_WHEEL_CURRENT) raw_current = MAX_WHEEL_CURRENT;
        if (raw_current < -MAX_WHEEL_CURRENT) raw_current = -MAX_WHEEL_CURRENT;

        chassis->wheel_currents[i] = raw_current;

        // 方向校准（乘以 ±1），保证电机转向正确
        chassis->wheel_currents[i] *= chassis->wheel_direction_calibration[i];
    }

    // 调试电流记录
    left_current_out  = (chassis->wheel_currents[0] + chassis->wheel_currents[1]) * 0.5f;
    right_current_out = (chassis->wheel_currents[2] + chassis->wheel_currents[3]) * 0.5f;

    // 发送电流指令（顺序：左前,左后,右后,右前）
    Motor_SendCurrent_Ex(&hfdcan1, MOTOR_3508_GROUP1,
                         chassis->wheel_currents[0],   // 左前
                         chassis->wheel_currents[1],   // 左后
                         chassis->wheel_currents[2],   // 右后
                         chassis->wheel_currents[3]);  // 右前
}

// ============================ 底盘任务函数 ============================
/**
 * @brief 差速底盘任务函数（在主线循环中周期性调用）
 * @param chassis 底盘控制结构体指针
 * @param vx      遥控器X轴（-100~100），控制前进/后退
 * @param vy      遥控器Y轴（-100~100），控制差速转弯（角速度主要部分）
 * @param vw      遥控器Z轴（-100~100），控制小陀螺自转叠加
 * @note 输入范围 -100~100，内部会转换为实际速度（米/秒，弧度/秒）
 *       最终角速度 ω = (vy/100)*MAX_ANGULAR_SPEED + (vw/100)*MAX_GYRO_SPEED
 *       三个通道同时生效，无需模式切换，小陀螺效果由 vw 实时提供。
 */
void Chassis_Task(Chassis_t *Chassis, Chassis_Mode mode, float vx, float vy, float vw) {
    if (Chassis == NULL) return;

    // 死区处理（摇杆小信号归零）
    if (fabsf(vx) < JOYSTICK_DEADZONE) vx = 0.0f;
    if (fabsf(vy) < JOYSTICK_DEADZONE) vy = 0.0f;
    if (fabsf(vw) < JOYSTICK_DEADZONE) vw = 0.0f;

    // 速度缩放：摇杆-100~100 → 实际速度
    float vx_scaled = vx / 100.0f * MAX_LINEAR_SPEED;               // 线速度 (m/s)
    float omega_vy  = vy / 100.0f * MAX_ANGULAR_SPEED;              // 差速转弯角速度 (rad/s)
    float omega_vw  = vw / 100.0f * MAX_GYRO_SPEED;                 // 小陀螺叠加角速度 (rad/s)
    float omega_total = omega_vy + omega_vw;                         // 总角速度 (rad/s)

    // 保存目标值（调试用）
    Chassis->Target_Vx = vx_scaled;
    Chassis->Target_Omega = omega_total;

    // 运动学解算，得到左右轮目标转速
    diff_solve(vx_scaled, omega_total,
               &Chassis->target_left_rpm,
               &Chassis->target_right_rpm);

    // 模式切换处理
    static Chassis_Mode last_mode = STOP_MODE;
    if (last_mode != mode) {
        last_mode = mode;
        Chassis->Mode = mode;
        if (mode == STOP_MODE) {
            Chassis->Target_Vx = 0.0f;
            Chassis->Target_Omega = 0.0f;
        }
    }

    if (mode) {
        switch (Chassis->Mode) {
            case NORMAL_MODE:
                Chassis->Target_Vx = vx_scaled;
                Chassis->Target_Omega = omega_total;
                // 运动学解算（车体坐标系）
                diff_solve(vx_scaled, omega_total,
                           &Chassis->target_left_rpm,
                           &Chassis->target_right_rpm);
                // PID控制 + 电流发送
                Chassis_Control(Chassis);
                break;
            case STOP_MODE:
            default:
                Motor_SendCurrent_Ex(&hfdcan1,MOTOR_3508_GROUP1, 0, 0, 0, 0);
                break;
        }
    } else {
        // 未使能，发送零电流
        Motor_SendCurrent_Ex(&hfdcan1,MOTOR_3508_GROUP1, 0, 0, 0, 0);
    }
}