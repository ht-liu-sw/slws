#ifndef ZEROERR_MOTOR_STRUCT_HPP__
#define ZEROERR_MOTOR_STRUCT_HPP__

#include "motor_template.h"

constexpr uint8_t PLACE_HOLDER8 = 0xFF;
struct zeroerr_communication_parameter : motor_communication_parameter {
    motor_item_can control_mode = {0x00, 0x4E, 0x00, 0x00, 0x00, 0x03};// 位置控制模式
    motor_item_can motion_mode = {0x00, 0x8D, 0x00, 0x00, 0x00, 0x01};// 目标绝对位置模式
    motor_item_can target_position = {0x00, 0x86, PLACE_HOLDER8, PLACE_HOLDER8, PLACE_HOLDER8, PLACE_HOLDER8};
    motor_item_can profile_velocity = {0x00, 0x8A ,PLACE_HOLDER8 ,PLACE_HOLDER8 ,PLACE_HOLDER8 ,PLACE_HOLDER8};
    motor_item_can profile_acceleration = {0x00, 0x88 ,PLACE_HOLDER8 ,PLACE_HOLDER8 ,PLACE_HOLDER8 ,PLACE_HOLDER8};
    motor_item_can profile_deceleration = {0x00, 0x89 ,PLACE_HOLDER8 ,PLACE_HOLDER8 ,PLACE_HOLDER8 ,PLACE_HOLDER8};
};
struct zeroerr_enable_motor_parameter:motor_communication_parameter{
    motor_item_can data{0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
};
struct zeroerr_disable_motor_parameter:motor_communication_parameter{
    motor_item_can data{0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
};
// 写入任意值即开始运动
struct zeroerr_start_motion_parameter: motor_communication_parameter{
    motor_item_can data{0x00, 0x83};
};
// 按减速度(profile_deceleration)减速到0
struct zeroerr_stop_motion_parameter: motor_communication_parameter{
    motor_item_can data{0x00, 0x84};
};
enum motion_status: uint8_t{
    MOTION_STOP = 0,        // 停止运动
    MOTIONING = 1,          // 运动中
    REPEAT_STOP = 3         // 重复停止中
};
struct zeroerr_read_motion_status: motor_communication_parameter{
    motor_item_can data{0x00, 0x20};
};

enum TARGET_REACH_STATUS: uint8_t{
    MOTOR_CLOSED = 0,                   // 电机关闭
    MOTOR_OPENED = 1,                   // 电机启动
    PROCESSING = 2,                     // 运行中
    WAITTING_LOCATING_TIME_CLEAR = 3,   // 等待定位时间清楚
    TARGET_REACH = 4                    // 目标到达
};
// 读取目标到达状态
struct zeroerr_read_target_reach_status: motor_communication_parameter{
    motor_item_can data{0x01, 0x0C};
};

// 读取当前位置
struct zeroerr_read_angle: motor_communication_parameter{
    motor_item_can data{0x00 ,0x02};
};


#endif // ZEROERR_MOTOR_STRUCT_HPP__
