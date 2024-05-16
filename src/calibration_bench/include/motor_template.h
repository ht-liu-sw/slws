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

struct motor_position_control_parameter {
  const double target_position_angle;
  const double profile_velocity_angle;
  const double profile_acceleration_angle;
  const double profile_deceleration_angle;
};
namespace default_motor_control_parameter{

  struct V90_A30_D30 : motor_position_control_parameter {
    V90_A30_D30(const double target_position)
        : motor_position_control_parameter{.target_position_angle =
                                               target_position,
                                           .profile_velocity_angle = 90.,
                                           .profile_acceleration_angle = 30.,
                                           .profile_deceleration_angle = 30.} {}
  };

  struct V45_A15_D15 : motor_position_control_parameter {

    V45_A15_D15(const double target_position)
        : motor_position_control_parameter{.target_position_angle =
                                               target_position,
                                           .profile_velocity_angle = 45.,
                                           .profile_acceleration_angle = 15.,
                                           .profile_deceleration_angle = 15.} {}
  };
  };

struct motor_communication_parameter {};

using motor_item_can = std::vector<uint8_t>;

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
  motor_template(const std::string& motor_name)
    : motor_name(motor_name), logger(rclcpp::get_logger(motor_name)) {
    
  }
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
