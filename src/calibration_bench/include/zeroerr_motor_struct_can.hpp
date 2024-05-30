#ifndef ZEROERR_MOTOR_STRUCT_HPP__
#define ZEROERR_MOTOR_STRUCT_HPP__

#include "motor_template.h"

namespace zeroerr_motor_default_frame {

constexpr uint8_t PLACE_HOLDER8 = 0xFF;

struct can_moment_control_msg : motor_communication_parameter {
  const struct cm2mccs_t {
    const can_data_t moment_control = {0x00, 0x4E, 0x00, 0x00, 0x00, 0x01};
    const can_data_t set_the_control_source_is_not_used = {0x01, 0x12, 0x00,
                                                           0x00, 0x00, 0x00};
    const can_data_t set_the_analog_quantity_for_internal = {0x01, 0xFD, 0x00,
                                                             0x00, 0x00, 0x00};
    const can_data_t set_target_current_to_0_mA = {0x01, 0xFE, 0x00,
                                                   0x00, 0x00, 0x00};
    const can_data_t set_the_maximum_speed_limit_to_10000cnt_s = {
        0x02, 0x04, 0x00, 0x00, 0x27, 0x10};
    const can_data_t motor_enable = {0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
  } change_motor_control_mode_to_moment_control_commands; // 顺序发送指令，使电机切换为`力矩控制`模式
  const cm2mccs_t &c2mccs =
      change_motor_control_mode_to_moment_control_commands;
  // other
  struct {
    const can_data_t motor_disable = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  } other_command;
};

struct can_break_control_msg_t : motor_communication_parameter {
  const can_data_t activate_break = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  const can_data_t release_break = {0x01, 0x4F};
  const can_data_t motor_enable = {0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
  const can_data_t motor_disable = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
} static can_break_control_msg;

struct can_position_control_msg : motor_communication_parameter {
  const can_data_t position_control = {0x00, 0x4E, 0x00,
                                       0x00, 0x00, 0x03}; // 位置控制模式
  const can_data_t motion_mode = {0x00, 0x8D, 0x00, 0x00,
                                  0x00, 0x01}; // 目标绝对位置模式, 不带往返

  /**
   * @brief 初始化数组
   *
   * @param target_array
   * 只可在成员[target_position, profile_velocity,
   * profile_acceleration, profile_deceleration]中选择
   * @param v1 顺序第1个数据
   * @param v2 顺序第2个数据
   * @param v3 顺序第3个数据
   * @param v4 顺序第4个数据
   */
  static void dynamic_array_init(can_data_t &target_array,
                                 const uint8_t v1,
                                 const uint8_t v2,
                                 const uint8_t v3,
                                 const uint8_t v4) {
    target_array.at(2) = v1;
    target_array.at(3) = v2;
    target_array.at(4) = v3;
    target_array.at(5) = v4;
  }
  can_data_t target_position = {0x00,          0x86,          PLACE_HOLDER8,
                                PLACE_HOLDER8, PLACE_HOLDER8, PLACE_HOLDER8};
  can_data_t profile_velocity = {0x00,          0x8A,          PLACE_HOLDER8,
                                 PLACE_HOLDER8, PLACE_HOLDER8, PLACE_HOLDER8};
  can_data_t profile_acceleration = {
      0x00, 0x88, PLACE_HOLDER8, PLACE_HOLDER8, PLACE_HOLDER8, PLACE_HOLDER8};
  can_data_t profile_deceleration = {
      0x00, 0x89, PLACE_HOLDER8, PLACE_HOLDER8, PLACE_HOLDER8, PLACE_HOLDER8};
};
static const can_data_t motor_enable_msg{0x01, 0x00, 0x00, 0x00, 0x00, 0x01};

static const can_data_t motor_disable_msg{0x01, 0x00, 0x00, 0x00, 0x00, 0x00};

// 写入任意值即开始运动
static const can_data_t motion_start_msg{0x00, 0x83};

// 按减速度(profile_deceleration)减速到0
static const can_data_t motion_stop_msg{0x00, 0x84};

enum motion_status : uint8_t {
  MOTION_STOP = 0, // 停止运动
  MOTIONING = 1,   // 运动中
  REPEAT_STOP = 3  // 重复停止中
};

static const can_data_t read_motion_statu{0x00, 0x20};

enum TARGET_REACH_STATUS : uint8_t {
  MOTOR_CLOSED = 0,                 // 电机关闭
  MOTOR_OPENED = 1,                 // 电机启动
  PROCESSING = 2,                   // 运行中
  WAITTING_LOCATING_TIME_CLEAR = 3, // 等待定位时间清楚
  TARGET_REACH = 4                  // 目标到达
};
// 读取目标到达状态
static const can_data_t read_target_reach_status_msg{0x01, 0x0C};

// 读取当前位置
static const can_data_t read_angle_msg{0x00, 0x02};

};     // namespace zeroerr_motor_default_frame
#endif // ZEROERR_MOTOR_STRUCT_HPP__
