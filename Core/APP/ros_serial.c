#include "ros_serial.h"
#include "chassis.h"
#include "serial.h"   // 确保 Serial_Printf 可用
#include "Grabber.h"
#include "../Hardware/elrs.h"
#include"usart.h"


#include "stm32h7xx_hal_uart.h"



#define SERIAL_UART huart7

// ==========================================================
// 内部变量定义
// ==========================================================
static uint8_t send_buff_rb[SEND_BUFF_SIZE + 1];
static lwrb_t  send_rb;
__attribute__((section(".ram_d1"))) static uint8_t usart_recv_buff[MAX_TYPE_SIZE];


static uint8_t recv_buff_rb[RECV_BUFF_SIZE + 1];
//static uint8_t usart_recv_buff[MAX_TYPE_SIZE];
static lwrb_t  recv_rb;
volatile uint32_t rx_count = 0;




float ros_vx = 0.0f;
float ros_vy = 0.0f;
float ros_vw = 0.0f;
uint8_t ros_arm_id = 0;
uint8_t ros_cylinder = 0;
uint8_t ros_action_id = 0;
volatile uint8_t ros_new_data_ready = 0;


static const uint8_t CRC8Table[] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};












uint8_t CRC8_Table(uint8_t* p, uint8_t lenth)
{
    uint8_t crc8 = 0;
    for (int i = 0; i < lenth; i++)
    {
        uint8_t value = p[i];
        uint8_t new_index = crc8 ^ value;
        crc8 = CRC8Table[new_index];
    }

    return crc8;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &SERIAL_UART) {
        // 将刚收到的1字节写入环形缓冲区
        lwrb_write(&recv_rb, usart_recv_buff, 1);
        // 重新启动下一次接收
        HAL_UART_Receive_IT(&SERIAL_UART, usart_recv_buff, 1);
    }
}

// ////
// /////////////////////////////
// ///
   void ROS_Serial_Init(){
       lwrb_init(&send_rb,send_buff_rb,sizeof(send_buff_rb));
       lwrb_init(&recv_rb,recv_buff_rb,sizeof(recv_buff_rb));
     lwrb_reset(&recv_rb);   //

     HAL_UART_Receive_IT(&SERIAL_UART, usart_recv_buff, 1);
   }


// void ROS_Serial_DMA_Send(){
//     lwrb_sz_t len = lwrb_get_linear_block_read_length(&send_rb);
//     HAL_UART_Transmit_DMA(&SERIAL_UART,lwrb_get_linear_block_read_address(&send_rb),len);
//     lwrb_skip(&send_rb, len);
// }






void ROS_Send_ACK(void) {
    uint8_t ack_buff[3] = {0xAA, 0x01, 0x55};
    HAL_UART_Transmit(&SERIAL_UART, ack_buff, 3, 100);
}



// ==========================================================
// 解析
// ==========================================================
void ROS_Serial_Data_CallBack(void) {
      if (lwrb_get_free(&recv_rb) == 0) {
      //Serial_Printf("in free=%d\r\n", lwrb_get_free(&recv_rb));  //
         lwrb_reset(&recv_rb);
         return;
     }


    while (lwrb_get_full(&recv_rb) >= PACKET_SIZE) {
        uint8_t frame[PACKET_SIZE];

        //
        lwrb_peek(&recv_rb, 0, frame, PACKET_SIZE);

        // 1. 找帧头
        if (frame[0] != FRAME_HEAD) {
            lwrb_skip(&recv_rb, 1);
            continue;
        }

        // 2. 校验长度 (偏移 1)
        if (frame[1] != PACKET_SIZE) {
          // Serial_Printf("[Len Err] recv=0x%02X, expect=0x%02X\r\n", frame[1], PACKET_SIZE);
            lwrb_skip(&recv_rb, 1);
            continue;
        }

        // 3. 校验帧尾 (偏移 22)
        if (frame[PACKET_SIZE - 1] != FRAME_END) {
           // Serial_Printf("[End Err] recv=0x%02X, expect=0x%02X\r\n", frame[PACKET_SIZE - 1], FRAME_END);
            lwrb_skip(&recv_rb, 1);
            continue;
        }

        // 4. 校验 CRC8 (计算范围: frame[2] 开始，共 19 个字节)
        uint8_t calc_crc = CRC8_Table(&frame[2], 19);
        if (calc_crc != frame[21]) {
          // Serial_Printf("[CRC Fail] calc=0x%02X, recv=0x%02X\r\n", calc_crc, frame[21]);
          // Serial_Printf("  Raw: ");
           // for (int i = 0; i < PACKET_SIZE; i++) {
           //     Serial_Printf("%02X ", frame[i]);
           // }
           // Serial_Printf("\r\n");
            lwrb_skip(&recv_rb, 1);
            continue;
        }

        // ================================================
        // 23
        // ================================================
        lwrb_read(&recv_rb, frame, PACKET_SIZE);


        RobotCommandPacket cmd;
        memcpy(&cmd, frame, PACKET_SIZE);



        // 保存数据到全局变量
        // ros_vx = cmd.linear_x;
        // ros_vw = cmd.angular_z;
        ros_vx = (cmd.linear_x*40.0f);
        ros_vw = (cmd.angular_z*40.0f);
        ros_arm_id = cmd.arm_id;
        ros_cylinder = cmd.cylinder_flag;
        ros_action_id = cmd.action_id;



       // Serial_Printf("[Parse OK] arm_id=%d, action=%d, linear_x=%.6f, angular_z=%.6f, ros_vx=%.2f, ros_vw=%.2f\r\n",
       //               cmd.arm_id, cmd.action_id, cmd.linear_x, cmd.angular_z, ros_vx, ros_vw);

        ///新数据标志
        ros_new_data_ready = 1;

    }
}





// ==========================================================
// 速度斜坡：限制 ros_vx/ros_vw 的变化速率，防止突变
// ros_vx 范围 -100~100，对应线速度 -2.5~2.5 m/s
// ros_vw 范围 -100~100，对应角速度 -6.0~6.0 rad/s
// 调用频率约 1kHz，每次变化量 = RAMP_STEP
// ==========================================================
static void ROS_Speed_Ramp(void) {
    #define RAMP_STEP_VX  0.1f    // 线速度步长，改小=更平滑
    #define RAMP_STEP_VW  0.2f    // 角速度步长，改小=更平滑
    static double ramp_target_vx = 0.0;
    static double ramp_target_vw = 0.0;

    // 收到新数据时，更新目标值（从当前位置向新目标斜坡）
    if (ros_new_data_ready) {
        ramp_target_vx = ros_vx;
        ramp_target_vw = ros_vw;
    }

    if (ros_vx < ramp_target_vx - RAMP_STEP_VX)
        ros_vx += RAMP_STEP_VX;
    else if (ros_vx > ramp_target_vx + RAMP_STEP_VX)
        ros_vx -= RAMP_STEP_VX;
    else
        ros_vx = ramp_target_vx;

    if (ros_vw < ramp_target_vw - RAMP_STEP_VW)
        ros_vw += RAMP_STEP_VW;
    else if (ros_vw > ramp_target_vw + RAMP_STEP_VW)
        ros_vw -= RAMP_STEP_VW;
    else
        ros_vw = ramp_target_vw;
}








void Process_ROS_Command(void) {



    // 1. 试解析缓冲区中的数据
    ROS_Serial_Data_CallBack();

    // 2. 速度斜坡平滑处理（
    ROS_Speed_Ramp();


    // ============================================
    // 2. 底盘连续控制（每5ms执行一次）
    // ============================================
    if (ros_arm_id == 0) {
        static uint32_t last_chassis_tick = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_chassis_tick >= 5) {
            last_chassis_tick = now;

            // 计算抬升轮状态
            // ros_cylinder: 1=开启, 2=关闭, 0=无变化
            uint8_t valve = 0;
            if (ros_cylinder == 1) {
                valve = 1;  // 开启抬升轮
            } else if (ros_cylinder == 2) {
                valve = 0;  // 关闭抬升轮
            }
            // 如果 ros_cylinder == 0，保持之前的状态
            // 但为了简单，我们使用静态变量保存上次状态
            static uint8_t last_valve = 0;
            if (ros_cylinder == 1 || ros_cylinder == 2) {
                last_valve = valve;
            }

            Chassis_Task(&Chassis, 1, ros_vx, 0, -ros_vw, last_valve);
        }
    }





    // 4. 只有真正执行了动作才发送ACK
    if (ros_new_data_ready) {
        ros_new_data_ready = 0;  // 清除新数据标志

        // 记录上次执行的状态，用于判断是否真正执行了动作
        static uint8_t last_arm_id = 0xFF;
        static uint8_t last_action_id = 0xFF;
        static uint8_t last_cylinder = 0xFF;

        bool action_executed = false;  // 标记是否真正执行了动作

        // ============================================
        // 机械臂1（舵机/夹爪）
        // ============================================
        if (ros_arm_id == 1) {
            // 只有动作ID变化时才执行
            if (ros_action_id != last_action_id) {
                switch (ros_action_id) {
                    case 1:
     Grabber_Move_To_Angle(GRABBER_ANGLE_HORIZONTAL, 1.0f, 5);     // 夹爪水平
                      action_executed = true;
                        break;
                    case 2:
                        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);  //
                        action_executed = true;
                        break;
                    case 3:
           Grabber_Move_To_Angle(GRABBER_ANGLE_VERTICAL, 1.0f, 5);   // 夹爪垂直
                        action_executed = true;
                        break;
                    case 4:
                        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);  // 关闭夹爪电磁阀
                        action_executed = true;
                        break;
                    default:
                        // 未知动作ID，不执行
                        break;
                }

                if (action_executed) {
                    last_action_id = ros_action_id;  // 更新记录
                }
            }
        }

        // ============================================
        // 机械臂2
        // ============================================
        else if (ros_arm_id == 2) {
            // 只有动作ID变化时才执行
            if (ros_action_id != last_action_id) {
                // 执行机械臂2的动作
                // Execute_Arm(2, ros_action_id);
                // 这里添加你的机械臂2执行代码

                action_executed = true;
                last_action_id = ros_action_id;  // 更新记录
            }
        }

        // ============================================
        // 气缸控制
        // ============================================
        if (ros_cylinder != last_cylinder) {
            if (ros_cylinder == 1) {

            //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // // 爬楼梯气缸开
                action_executed = true;
            } else if (ros_cylinder == 2) {
           // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);  // 爬楼梯气缸关
                action_executed = true;
            }

            if (action_executed) {
                last_cylinder = ros_cylinder;  // 更新记录
            }
        }

        // ============================================
        // 只有真正执行了动作才发送ACK确认
        // ============================================
        if (action_executed) {
            ROS_Send_ACK();  // 发送执行完成确认
            // 调试输出（可选）
            // Serial_Printf("[ACK] arm=%d, action=%d, cylinder=%d\r\n",
            //               ros_arm_id, ros_action_id, ros_cylinder);
        }

        // 更新机械臂ID记录（用于下次比较）
        if (ros_arm_id != last_arm_id) {
            last_arm_id = ros_arm_id;
        }
    }
}

//总控制
void car_control(void) {
    //manual control mode
    if (remoter.key.SB == 1) {
         Chassis_Task(&Chassis,remoter.key.SB,remoter.joy.r_x,remoter.joy.r_y,remoter.joy.l_y,remoter.key.SE);
       // Grabber_Task(remoter.key.SF, 2.0f, 4);
    }
    else if (remoter.key.SB == 2) {
        Process_ROS_Command();
    }
}
