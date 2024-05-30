#pragma once

//#include "zeroerr_motor_can.h"

#include "can_comm.hpp"
#include "motor_template.h"
#include "zeroerr_motor_can.h"
#include <cmath>
#include <memory>
#include <optional>
#include <ratio>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace {
using namespace default_motor_control_parameter;
};
/**
 * @brief 标定电机的控制
 *
 */
class calibration_motors_can{
  using frame_id_t = uint32_t;

public:
  using init_param_t = struct init_param_t {
    const std::string calibration_bench_name;
    const std::string motor_xy_name;
    const std::string motor_xz_name;
    const frame_id_t motor_xy_can_id;
    const frame_id_t motor_xz_can_id;
    const std::string can_device_name;
  };

public:
  calibration_motors_can(const init_param_t &param)
      :init_param(param), logger(rclcpp::get_logger(param.calibration_bench_name)) {}

  /**
   * @brief 总体初始化
   *
   * @return true
   * @return false
   */
  auto init() -> bool { return init_can_device() && init_motors_handle(); }

  ~calibration_motors_can() { disconnect_motors(); };

  /**
   * @brief XY电机同步移动，并等待wait_time_ms时间
   *
   * @param param
   * @param wait_time_ms 单位ms
   */
  void motor_xy_sync_position_control(const mpcp_t &&param) {
    const auto offseted_param =
        param.fork(param.target_position_angle +
                   motors_offset_status.motor_xz_zero_offset);
    motor_xy_handle->position_control(offseted_param, true);
    get_motors_position();
  }

  /**
   * @brief XZ电机同步移动，并等待wait_time_ms时间
   *
   * @param param
   * @param wait_time_ms
   */
  void motor_xz_sync_position_control(const mpcp_t &&param) {
    const auto offseted_param =
        param.fork(param.target_position_angle +
                   motors_offset_status.motor_xz_zero_offset);
    motor_xz_handle->position_control(offseted_param, true);
    get_motors_position();
  }

  /**
   * @brief 参数可设置
   *
   * @param p_xy target_position无效，使用v、a、d作为运动参数
   * @param p_xz target_position无效，使用v、a、d作为运动参数
   */
  void motors_async_homing(const mpcp_t &p_xy, const mpcp_t &p_xz) {
    motor_xz_handle->position_control(
        p_xy.fork(motors_offset_status.motor_xy_zero_offset), false);
    motor_xy_handle->position_control(
        p_xz.fork(motors_offset_status.motor_xz_zero_offset), false);
    motor_xy_handle->wait_motion_finish();
    motor_xz_handle->wait_motion_finish();
  }

  /**
   * @brief 设置进行力标定时MotorXY每步旋转的角度
   *
   * @param step
   */
  void set_motorxy_step_angle(const angle_t step) {
    if (std::fmod(360.0, step) != 0.0) {
      RCLCPP_INFO(logger, "step_angle is invalid: %lf", step);
      throw std::logic_error(std::string("step_angle is invalid: ") +
                             std::to_string(step));
    }
  }

  /**
   * @brief 使用`motion_param`的参数返回
   *
   * @param motion_param 使用v,a,d参数进行运动
   */
  void motor_xy_homing(const mp_t &motion_param) {
    motor_xy_sync_position_control(
        PVAD(motors_offset_status.motor_xy_zero_offset, motion_param));
  }

  /**
   * @brief 使用`motion_param`的参数返回
   *
   * @param motion_param 使用v,a,d参数进行运动
   */
  void motor_xz_homing(const mp_t &motion_param) {
    motor_xz_sync_position_control(
        PVAD(motors_offset_status.motor_xz_zero_offset, motion_param));
  }

  /**
   * @brief 使motor_xy切换为0N的力矩控制;
   *
   */
  void motor_xy_switch_to_moment_control() {
    motor_xz_handle->switch_to_0N_moment_control();
  }

  void motor_xy_release_break() { motor_xy_handle->release_break(); }

  void motor_xy_activate_break() { motor_xy_handle->activate_break(); }

  using motors_position_t = struct {
    angle_t motor_xy_source_angle;
    angle_t motor_xz_source_angle;
    angle_t motor_xy_offseted_angle;
    angle_t motor_xz_offseted_angle;
  };
  using opt_motors_position_t = std::optional<motors_position_t>;

  // 更新电机角度
  auto get_motors_position() -> opt_motors_position_t {
    const auto angle_xy_opt = motor_xy_handle->read_angle();
    const auto angle_xz_opt = motor_xz_handle->read_angle();
    if (angle_xy_opt.has_value() && angle_xz_opt.has_value()) {
      const auto angle_xy = angle_xy_opt.value();
      const auto angle_xz = angle_xz_opt.value();
      motors_position_t res{
          .motor_xy_source_angle = angle_xy,
          .motor_xz_source_angle = angle_xz,
          .motor_xy_offseted_angle =
              angle_xy - motors_offset_status.motor_xy_zero_offset,
          .motor_xz_offseted_angle =
              angle_xz - motors_offset_status.motor_xz_zero_offset};
      return std::make_optional(std::move(res));
    }
    return std::nullopt;
  }

  void update_motors_zero_offset(const double motor_xy_zero_angle,
                                 const double motor_xz_zero_angle) {
    RCLCPP_INFO(logger,
                "%s | set motor_xy_offset to %lf, set motor_xz_offset to %lf",
                __func__, motor_xy_zero_angle, motor_xz_zero_angle);
    this->motors_offset_status.motor_xy_zero_offset = (motor_xy_zero_angle);
    this->motors_offset_status.motor_xz_zero_offset = (motor_xz_zero_angle);
  }

  bool connect_motors() {
    // TODO: 检查 连接状态
    return true;
  }

  bool disconnect_motors() { return true; }

private:
  /**
   * @brief 初始化电机组
   *
   * @return true
   * @return false
   */
  auto init_motors_handle() -> bool {
    const auto &param = init_param;
    motor_xy_handle = std::make_unique<zeroerr_motor_can>(
        param.motor_xy_name, param.motor_xy_can_id, can_handle);
    motor_xz_handle = std::make_unique<zeroerr_motor_can>(
        param.motor_xz_name, param.motor_xz_can_id, can_handle);
    return true;
  };

  /**
   * @brief 初始化CAN设备
   *
   * @return true
   * @return false
   */
  auto init_can_device() -> bool {
    RCLCPP_INFO(logger, "init_params:{can_device_name: %s}",
                init_param.can_device_name.c_str());
    can_handle = std::make_shared<can_comm>();
    return can_handle->init(init_param.can_device_name);
  }

private:
  std::unique_ptr<zeroerr_motor_can> motor_xy_handle;
  std::unique_ptr<zeroerr_motor_can> motor_xz_handle;
  uint32_t data_capture_wait_time = 8'000; // 获取数据的等待时间，unit:ms
  const uint32_t data_capture_before_wait_time =
      300; // 获取数据前等待电机稳定的时间，unit:ms
  struct motors_offset_status_t {
    bool inited = false;
    double motor_xy_zero_offset = 0.0;
    double motor_xz_zero_offset = 0.0;
  } motors_offset_status;

private:
  init_param_t init_param;
  rclcpp::Logger logger;
  std::atomic<bool> init_finish{false};
  std::shared_ptr<can_comm> can_handle;
};
