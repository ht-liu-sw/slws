#pragma once
#ifndef ZEROERR_MOTOR_H__
#define ZEROERR_MOTOR_H__

#include "motor_template.h"
#include "zeroerr_motor_struct_can.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp/logger.hpp>
#include <type_traits>


#include "ECanVci.h"

constexpr UINT USBCAN1 = 3;
constexpr UINT USBCAN2 = 4;

using can_info_t = struct can_info_t {
  const UINT DeviceType = USBCAN1; // 单通道
  const UINT DeviceInd = 0;
  const UINT CANChannel = 0;
};
using frame_id_t = uint16_t;

constexpr frame_id_t zeroerr_can_frame_offset = 0x0640;
/**
 * @brief CAN的零差云控电机接口，与CAN__ID绑定
 *
 */
class zeroerr_motor_can : public motor_template {
public:
  zeroerr_motor_can(const std::string &motor_name, const frame_id_t can_id,
                    const can_info_t can_info)
      : motor_template(motor_name), transmit_frame_id(zeroerr_can_frame_offset + can_id), recv_frame_id(zeroerr_can_frame_offset + can_id),
        can_info(can_info) {}
  bool position_control(const motor_position_control_parameter &param,
                        bool sync) override;

  std::shared_ptr<zeroerr_communication_parameter>
  dump_zeroerr_communication_parameter(const int32_t target_pose,
                                       const int32_t profile_velocity,
                                       const int32_t profile_acceleration,
                                       const int32_t profile_deceleration);

  o_angle_t read_angle() override;

  bool wait_motion_finish();

private:
  bool motor_enable();
  bool motor_disable();
  bool motion_start();
  bool motion_stop();
  bool motion_quick_stop();

private:
  /**
   * @brief 向电机写入motor_communication_parameter
   *
   * @param param
   * @return true
   * @return false
   */
  bool local_transmit(const motor_item_can &param);

  /**
   * @brief 读取回复
   *
   * @return std::vector<uint8_t> 返回数据数组
   */
  std::optional<std::vector<uint8_t>> local_recv();

private:
  const frame_id_t transmit_frame_id;
  const frame_id_t recv_frame_id;
  can_info_t can_info;
};

#endif // ZEROERR_MOTOR_H__
