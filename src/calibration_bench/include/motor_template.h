#ifndef MOTOR_TEMPLATE_H__
#define MOTOR_TEMPLATE_H__

#include <atomic>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <memory>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

// 运动参数，包括`速度`、`加速度`、`减速度`
using mp_t = struct motion_parameter {
  const double profile_velocity_angle;
  const double profile_acceleration_angle;
  const double profile_deceleration_angle;
};

using mpcp_t = struct motor_position_control_parameter {
  const double target_position_angle;
  const mp_t motion_parameter;

  /**
   * @brief 使用新的 target_position 与自身的其它参数生成一个新的对象
   *
   * @param new_target_position_angle
   */
  struct motor_position_control_parameter
  fork(const double new_target_position_angle) const {
    return motor_position_control_parameter{
        .target_position_angle = new_target_position_angle,
        .motion_parameter = motion_parameter};
  };
};

namespace default_motor_control_parameter {
/**
 * @brief
 *
 * @param position angle单位
 * @param velocity angle/s
 * @param acceleration angle/s^2
 * @param deceleration angle/s^2
 * @return mpcp_t
 */
constexpr auto PVAD(const double position, const double velocity,
                    const double acceleration, const double deceleration)
    -> mpcp_t {
  return mpcp_t{.target_position_angle = position,
                .motion_parameter =
                    mp_t{.profile_velocity_angle = velocity,
                         .profile_acceleration_angle = acceleration,
                         .profile_deceleration_angle = deceleration}};
};
constexpr auto PVAD(const double position, const mp_t &mp_param) -> mpcp_t {
  return mpcp_t{.target_position_angle = position,
                .motion_parameter = mp_param};
};

constexpr auto VAD(const double velocity, const double acceleration,
                   const double deceleration) -> mp_t {
  return mp_t{.profile_velocity_angle = velocity,
              .profile_acceleration_angle = acceleration,
              .profile_deceleration_angle = deceleration};
}

}; // namespace default_motor_control_parameter

struct motor_communication_parameter {};

using can_data_t = std::vector<uint8_t>;

using angle_t = double;
using o_angle_t = std::optional<angle_t>;

class motor_template {

protected:
  // logger handle
  const std::string motor_name;
  rclcpp::Logger logger;

public:
  /**
   * @brief Construct a new motor template object
   *
   * @param motor_name 与日志相关，用于设置logger名称
   */
  motor_template(const std::string &motor_name)
      : motor_name(motor_name), logger(rclcpp::get_logger(motor_name)) {}
  using Levels = rclcpp::Logger::Level;

  /**
   * @brief 命令电机运动，运动完成后返回
   *
   * @return true
   * @return false
   */
  virtual bool position_control(const motor_position_control_parameter &param,
                                bool sync) = 0;

  virtual o_angle_t read_angle() = 0;

public:
  bool is_inited();

  void set_zero_angle(const angle_t angle);

protected:
  std::atomic<bool> init_finish = {false};
};

#endif // MOTOR_TEMPLATE_H__
