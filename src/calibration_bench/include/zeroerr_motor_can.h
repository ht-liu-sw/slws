#ifndef __ZEROERR_MOTOR_CAN_H__
#define __ZEROERR_MOTOR_CAN_H__

#pragma once
#ifndef ZEROERR_MOTOR_H__
#define ZEROERR_MOTOR_H__

#include "can_comm.hpp"
#include "motor_template.h"
#include "zeroerr_motor_struct_can.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp/logger.hpp>
#include <type_traits>

using frame_id_t = uint16_t;
using frame_raw_data_t = std::vector<uint8_t>;

constexpr frame_id_t zeroerr_can_frame_offset = 0x0640;
/**
 * @brief CAN的零差云控电机接口，与CAN__ID绑定
 *
 */
class zeroerr_motor_can : public motor_template {
public:
  zeroerr_motor_can(const std::string &motor_name, const frame_id_t can_id,
                    std::shared_ptr<can_comm> can_handle)
      : motor_template(motor_name),
        transmit_frame_id(zeroerr_can_frame_offset + can_id),
        recv_frame_id(zeroerr_can_frame_offset + can_id),
        can_handle(can_handle) {}
  bool position_control(const motor_position_control_parameter &param,
                        bool sync) override;

  std::shared_ptr<zeroerr_motor_default_frame::can_position_control_msg>
  dump_can_position_control_msg(const int32_t target_pose,
                                const int32_t profile_velocity,
                                const int32_t profile_acceleration,
                                const int32_t profile_deceleration);

  bool switch_to_0N_moment_control();

  bool release_break();
  bool activate_break();

  auto write_single_warp(const can_data_t &item, const std::string &part_msg)
      -> bool;

  o_angle_t read_angle() override;

  void clean_recv_buffer();

  bool wait_motion_finish();

private:
  static constexpr auto ZEROERR_CAN_TAIL_NORMAL = 0x3E;
  static constexpr auto ZEROERR_CAN_TAIL_ERROR = 0x80;

  bool motor_enable();
  bool motor_disable();
  bool motion_start();
  bool motion_stop();
  bool motion_quick_stop();

private:
  std::optional<can_frame_t> sync_transmit_template(const can_data_t &msg);

private:
  const frame_id_t transmit_frame_id;
  const frame_id_t recv_frame_id;
  std::shared_ptr<can_comm> can_handle;
};

#endif // ZEROERR_MOTOR_H__

#endif // __ZEROERR_MOTOR_CAN_H__