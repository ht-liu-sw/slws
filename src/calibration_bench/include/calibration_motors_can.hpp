#pragma once

#include "ECanVci.h"
#include "zeroerr_motor_can.h"

#include "motor_template.h"
#include <cmath>
#include <memory>
#include <optional>

namespace {
using namespace default_motor_control_parameter;
};
/**
 * @brief 标定用两电机的控制
 *
 * @tparam MotorXY 初始位姿下，旋转轴为Z轴的电机的控制（旋转轴垂直于XY平面）
 * @tparam MotorXZ 初始位姿下，旋转轴为Y轴的电机的控制（旋转轴垂直于XZ平面）
 */
class calibration_motors_can {

public:
  using init_param_t = struct init_param_t {
    const std::string calibration_bench_name;
    const std::string motor_xy_name;
    const std::string motor_xz_name;
    can_info_t can_info;
    const frame_id_t motor_xy_can_id;
    const frame_id_t motor_xz_can_id;
  };

public:
  calibration_motors_can(const init_param_t &param)
      : logger(rclcpp::get_logger(param.calibration_bench_name)),
        init_param(param) {}
  auto init_motors_handle() -> bool {
    const auto &param = init_param;
    motor_xy_handle = std::make_unique<zeroerr_motor_can>(
        param.motor_xy_name, param.motor_xy_can_id, param.can_info);
    motor_xz_handle = std::make_unique<zeroerr_motor_can>(
        param.motor_xz_name, param.motor_xz_can_id, param.can_info);
    return true;
  };

  bool init_can_device() {
    const auto &paras = init_param.can_info;
    RCLCPP_INFO(logger,
                "init_params:{CANChannel:%d, DeviceInd:%d,  DeviceType:%d}",
                paras.CANChannel, paras.DeviceInd, paras.DeviceType);
    // 预先关闭设备
    auto init_can_ret = [this, paras]() -> bool {
      // if(CloseDevice(paras.DeviceType, paras.DeviceInd) == STATUS_OK){
      //   RCLCPP_INFO(logger, "Close CAN Device Success");
      // }
      auto open_device_wrap = [&paras, this]() -> bool {
        if (OpenDevice(paras.DeviceType, paras.DeviceInd, 0) != STATUS_OK) {
          RCLCPP_WARN(logger, "err.0 打开设备失败");
          return false;
          CloseDevice(paras.DeviceType, paras.DeviceInd);
        }
        return true;
      };
      bool open_device_ret = false;
      constexpr auto RETRY_TIMES = 10;
      for (int i = 0; i < RETRY_TIMES; ++i) {
        open_device_ret = open_device_wrap();
        if (open_device_ret)
          break;
        else {
          RCLCPP_WARN(logger, "retry Open CAN Device (%d/%d)", (i + 1),
                      RETRY_TIMES);
        }
      }

      INIT_CONFIG init_config;
      memset(&init_config, 0, sizeof(init_config));
      init_config.AccCode = 0;
      init_config.AccMask = 0xffffff; //不滤波

      init_config.Filter = 0;
      init_config.Timing0 = 0x00;
      init_config.Timing1 = 0x14; //波特率 1000k

      init_config.Mode = 0; //正常模式
      if (InitCAN(paras.DeviceType, paras.DeviceInd, paras.CANChannel,
                  &init_config) != STATUS_OK) {
        RCLCPP_WARN(logger, "can初始化失败");
        CloseDevice(paras.DeviceType, paras.DeviceInd);
        return false;
      }

      if (StartCAN(paras.DeviceType, paras.DeviceInd, paras.CANChannel) !=
          STATUS_OK) {
        RCLCPP_WARN(logger, "2打开通道失败");
        CloseDevice(paras.DeviceType, paras.DeviceInd);
        return false;
      }
      return true;
    }();
    if (init_can_ret == false) {
      RCLCPP_WARN(logger, "zeroerr_motor_can init false");
    } else {
      RCLCPP_INFO(logger, "zeroerr_motor_can init success");
    }
    init_finish = init_can_ret;
    return init_can_ret;
  }

  ~calibration_motors_can() { disconnect_motors(); };

  /**
   * @brief XY电机同步移动，并等待wait_time_ms时间
   *
   * @param param
   * @param wait_time_ms 单位ms
   */
  void motor_xy_sync_position_control(const V90_A30_D30 &&param) {
    V90_A30_D30 offseted_param(param.target_position_angle +
                               motor_xy_zero_offset);
    motor_xy_handle->position_control(offseted_param, true);
    this->get_motors_position();
  }

  /**
   * @brief XZ电机同步移动，并等待wait_time_ms时间
   *
   * @param param
   * @param wait_time_ms
   */
  void motor_xz_sync_position_control(const V45_A15_D15 &&param) {
    V45_A15_D15 offseted_param(param.target_position_angle +
                               motor_xz_zero_offset);
    motor_xz_handle->position_control(offseted_param, true);
    get_motors_position();
  }

  void motors_async_homing() {
    motor_xz_handle->position_control(V45_A15_D15(motor_xz_zero_offset), false);
    motor_xy_handle->position_control(V90_A30_D30(motor_xy_zero_offset), false);
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

  void motor_xy_homing() { motor_xy_sync_position_control(V90_A30_D30(0)); }

  void motor_xz_homing() { motor_xz_sync_position_control(V45_A15_D15(0)); }

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
          .motor_xy_offseted_angle = angle_xy - motor_xy_zero_offset,
          .motor_xz_offseted_angle = angle_xz - motor_xz_zero_offset};
      return std::make_optional(std::move(res));
    }
    return std::nullopt;
  }

  void update_motors_zero_offset(const double motor_xy_zero_angle,
                                 const double motor_xz_zero_angle) {
    this->motor_xy_zero_offset = (motor_xy_zero_angle);
    this->motor_xz_zero_offset = (motor_xz_zero_angle);
  }

  bool connect_motors() {
    // TODO: 检查 连接状态
    return true;
  }

  bool disconnect_motors() {
    const auto &paras = init_param.can_info;
    CloseDevice(paras.DeviceType, paras.DeviceInd);
    return true;
  }

private:
  std::unique_ptr<zeroerr_motor_can> motor_xy_handle;
  std::unique_ptr<zeroerr_motor_can> motor_xz_handle;
  uint32_t data_capture_wait_time = 8'000; // 获取数据的等待时间，unit:ms
  const uint32_t data_capture_before_wait_time =
      300; // 获取数据前等待电机稳定的时间，unit:ms
  std::atomic<double> motor_xy_zero_offset, motor_xz_zero_offset;

private:
  rclcpp::Logger logger;
  std::atomic<bool> init_finish{false};
  init_param_t init_param;
};
