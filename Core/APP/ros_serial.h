#ifndef __ROS_SERIAL_H
#define __ROS_SERIAL_H

#include "main.h"    // 包含 HAL 库头文件
#include "../Hardware/lwrb.h"



#define PACKET_SIZE       23      // 数据包总长度
#define FRAME_HEAD        0xAA    // 固定帧头
#define FRAME_END         0x55    // 固定帧尾

#define SEND_BUFF_SIZE    256     // 发送环形缓冲区大小
#define RECV_BUFF_SIZE    512     // 接收环形缓冲区大小
#define MAX_TYPE_SIZE     256     // 单次接收最大长度

// ========================================================
// 1 字节对齐
// ==========================================================
#pragma pack(push, 1)
typedef struct {
    uint8_t  frame_head;      // 0: 帧头 0xAA
    uint8_t  pack_len;        // 1: 长度 0x17 (23)
    double   linear_x;        // 2~9: 线速度
    double   angular_z;       // 10~17: 角速度
    uint8_t  arm_id;          // 18: 机械臂ID (0底盘, 1机械臂1, 2机械臂2)
    uint8_t  cylinder_flag;   // 19: 气缸 (0无, 1抬起, 2放下)
    uint8_t  action_id;       // 20: 动作ID
    uint8_t  pack_crc8;       // 21: CRC校验
    uint8_t  frame_end;       // 22: 帧尾 0x55
} RobotCommandPacket;
#pragma pack(pop)


extern float ros_vx;
extern float ros_vy;
extern float ros_vw;
extern uint8_t ros_arm_id;
extern uint8_t ros_cylinder;
extern uint8_t ros_action_id;
extern volatile uint32_t rx_count;

void ROS_Serial_Init(void);
void ROS_Serial_DMA_Send(void);
void ROS_Serial_Data_CallBack(void);
void Process_ROS_Command(void);
void car_control(void);
void ROS_Send_ACK(void);
#endif /* __ROS_SERIAL_H */




