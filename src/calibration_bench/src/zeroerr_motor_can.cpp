#include "zeroerr_motor_can.h"
#include "can_comm.hpp"
#include "zeroerr_motor_struct_can.hpp"
#include <algorithm>
#include <asm-generic/errno-base.h>
#include <bit>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

namespace {
using std::this_thread::sleep_for;
using namespace zeroerr_motor_default_frame;
} // namespace

bool zeroerr_motor_can::position_control(
    const motor_position_control_parameter &param, bool sync_motion) {
  RCLCPP_INFO(logger, "%s target_position(angle): %lf, sync_motion: %s",
              __func__, param.target_position_angle,
              (sync_motion ? "true" : "false"));
  auto angle2coder = [](const double angle) {
    constexpr int32_t coder_cnt = (1l << 19);
    return angle / 360.0 * coder_cnt;
  };

  [this, angle2coder](const motor_position_control_parameter &param) {
    // 设置参数
    const auto target_position_angle = param.target_position_angle;
    auto cp = dump_can_position_control_msg(
        angle2coder(target_position_angle),
        angle2coder(param.motion_parameter.profile_velocity_angle),
        angle2coder(param.motion_parameter.profile_acceleration_angle),
        angle2coder(param.motion_parameter.profile_deceleration_angle));

    write_single_warp(cp->motion_mode, "set motion_mode");
    write_single_warp(cp->profile_velocity, "set profile_velocity");
    write_single_warp(cp->profile_acceleration, "set profile_acceleration");
    write_single_warp(cp->profile_deceleration, "set profile_deceleration");
    write_single_warp(cp->target_position, "set target_position");

    motor_enable();
    // 开始运动
    motion_start();
  }(param);
  if (sync_motion) {
    auto motion_ret = wait_motion_finish();
    return motion_ret;
  }
  return true;
}

auto zeroerr_motor_can::write_single_warp(const can_data_t &item,
                                          const std::string &part_msg) -> bool {
  // write_single
  auto result = sync_transmit_template(item);

  if (result.has_value()) {
    RCLCPP_INFO(logger, "%s", (part_msg + " success").c_str());
    return true;
  } else {
    RCLCPP_INFO(logger, "%s", (part_msg + " failure").c_str());
    return false;
  }
}

std::shared_ptr<can_position_control_msg>
zeroerr_motor_can::dump_can_position_control_msg(
    const int32_t target_position, const int32_t profile_velocity,
    const int32_t profile_acceleration, const int32_t profile_deceleration) {
  // static_assert(expression, message);
  auto split_int32_to = [](int32_t val, can_data_t &vec) {
    auto ptr = reinterpret_cast<uint8_t *>(&val);
    // can_position_control_msg相关，替换掉PLACE_HOLDER8
    if (std::endian::native == std::endian::big) {
      can_position_control_msg::dynamic_array_init(vec, ptr[0], ptr[1], ptr[2],
                                                   ptr[3]);
    } else if (std::endian::native == std::endian::little) {
      can_position_control_msg::dynamic_array_init(vec, ptr[3], ptr[2], ptr[1],
                                                   ptr[0]);
    }
  };

  auto result = std::make_shared<can_position_control_msg>();
  {
    split_int32_to(target_position, result->target_position);
    split_int32_to(profile_velocity, result->profile_velocity);
    split_int32_to(profile_acceleration, result->profile_acceleration);
    split_int32_to(profile_deceleration, result->profile_deceleration);
  }
  return result;
}

/**
 * @brief 清除接收缓存
 *
 */
void zeroerr_motor_can::clean_recv_buffer() {
  // 一直读取缓存，直到报错
  do {
    const auto &&[data, error_info] = can_handle->read_frame();
    if (!data.has_value() && error_info.err_no == EAGAIN) {
      break;
    } else if (error_info.err_no == 0)
      continue;
    else {
      RCLCPP_WARN(logger, "socketcan recv has a error(%d): %s",
                  error_info.err_no, error_info.err_msg.c_str());
    }

  } while (true);
  RCLCPP_INFO(logger, "%s", __func__);
}

o_angle_t zeroerr_motor_can::read_angle() {
  // TODO: 清除缓存
  clean_recv_buffer();

  auto recv_data = sync_transmit_template(read_angle_msg);
  if (recv_data.has_value()) {
    const auto data = recv_data->data;
    const auto frame_error_number = data[recv_data->can_dlc - 1];
    if (frame_error_number == ZEROERR_CAN_TAIL_NORMAL) {
      // zeroerr 正常结束
      int32_t res = 0;
      res |= data[0];
      res <<= 8;
      res |= data[1];
      res <<= 8;
      res |= data[2];
      res <<= 8;
      res |= data[3];

      constexpr int32_t coder_cnt = (1 << 19);
      const double original_angle = (((double)res) / coder_cnt) * 360.0;
      return {original_angle};
    } else if (frame_error_number == ZEROERR_CAN_TAIL_ERROR) {
      RCLCPP_WARN(logger, "收到异常");
      return std::nullopt;
    } else {
      RCLCPP_ERROR(logger, "严重错误，接收到未知数据末尾: 0x%x",
                   frame_error_number);
      return std::nullopt;
    }
  } else {
    RCLCPP_WARN(logger, "%s 数据接收失败", __func__);
    return std::nullopt;
  }
}

// 刹车动作安全时间
constexpr auto SAFE_MOTOR_BREAK_WAIT_TIME = std::chrono::milliseconds(200);

bool zeroerr_motor_can::motor_enable() {
  RCLCPP_DEBUG(logger, __func__);
  const auto recv_frame = sync_transmit_template(motor_enable_msg);
  if (recv_frame.has_value()) {
    // 电机使能后等待安全时间，刹车动作时间约150ms, 取200ms
    sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
    return true;
  } else {
    // 数据没有收到
    RCLCPP_WARN(logger, "接收数据失败，数据为空");
    return false;
  }
}
bool zeroerr_motor_can::motor_disable() {
  RCLCPP_DEBUG(logger, __func__);
  const auto recv_frame = sync_transmit_template(motor_disable_msg);
  if (recv_frame.has_value()) {
    // 电机使能后等待安全时间，刹车动作时间约150ms, 取200ms
    sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
    return true;
  } else {
    // 数据没有收到
    RCLCPP_WARN(logger, "接收数据失败，数据为空");
    return false;
  }
}

bool zeroerr_motor_can::motion_start() {
  RCLCPP_DEBUG(logger, __func__);
  const auto recv_frame = sync_transmit_template(motion_start_msg);
  if (recv_frame.has_value()) {
    // 电机使能后等待安全时间，刹车动作时间约150ms, 取200ms
    sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
    return true;
  } else {
    // 数据没有收到
    RCLCPP_WARN(logger, "接收数据失败，数据为空");
    return false;
  }
}
bool zeroerr_motor_can::motion_stop() {
  RCLCPP_DEBUG(logger, __func__);
  const auto recv_frame = sync_transmit_template(motion_stop_msg);
  if (recv_frame.has_value()) {
    // 电机使能后等待安全时间，刹车动作时间约150ms, 取200ms
    sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
    return true;
  } else {
    // 数据没有收到
    RCLCPP_WARN(logger, "接收数据失败，数据为空");
    return false;
  }
}

bool zeroerr_motor_can::wait_motion_finish() {
  RCLCPP_DEBUG(logger, "motion start");
  // 等待运动结束
  auto motion_ret = [this]() -> bool {
    auto motion_finish = false;
    do {
      auto recv_data = sync_transmit_template(read_target_reach_status_msg);
      if (recv_data.has_value()) {
        motion_finish =
            (TARGET_REACH_STATUS::TARGET_REACH == recv_data->data[3]);
        // 检查周期 recv_data_time + 50ms
        sleep_for(std::chrono::milliseconds(50));
      } else {
        RCLCPP_WARN(logger, "wait_motion_finish: communication failure");
        return false;
      }
    } while (!motion_finish);
    RCLCPP_INFO(logger, "motion finish");
    return true;
  }();
  return motion_ret;
}

std::optional<can_frame_t>
zeroerr_motor_can::sync_transmit_template(const can_data_t &msg) {
  auto frame = can_comm::dump_can_frame(transmit_frame_id, msg);
  std::memcpy(frame.data, msg.data(), msg.size());
  can_handle->send_frame(frame);
  // 读取反馈
  const auto &[recv_frame, error_info] = can_handle->read_frame();
  if (error_info.err_no != 0) {
    RCLCPP_INFO(logger, "%s | read_frame error, msg: %s", __func__,
                error_info.err_msg.c_str());
  }
  return recv_frame;
}

bool zeroerr_motor_can::release_break() {
  const auto ret =
      write_single_warp(can_break_control_msg.release_break, "release break");
  std::this_thread::sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
  return ret;
}

bool zeroerr_motor_can::activate_break() {
  const auto ret =
      write_single_warp(can_break_control_msg.activate_break, "activate break");
  std::this_thread::sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
  return ret;
}

bool zeroerr_motor_can::switch_to_0N_moment_control() {

  const zeroerr_motor_default_frame::can_moment_control_msg msgs{};
  bool ret = true;
  ret = ret && write_single_warp(msgs.c2mccs.moment_control,
                                 "switch to moment_control");
  ret = ret && write_single_warp(msgs.c2mccs.set_the_control_source_is_not_used,
                                 "set the control source is not used");
  ret =
      ret && write_single_warp(msgs.c2mccs.set_the_analog_quantity_for_internal,
                               "set the analog quantity for internal");
  ret = ret && write_single_warp(msgs.c2mccs.set_target_current_to_0_mA,
                                 "set target current to 0 mA");
  ret = ret &&
        write_single_warp(msgs.c2mccs.set_the_maximum_speed_limit_to_10000cnt_s,
                          "set the maximum speed limit to 10000cnt/s");
  ret = ret && write_single_warp(msgs.c2mccs.motor_enable, "motor enable");
  return ret;
}
