#include "calibration_bench.hpp"
#include "motor_template.h"
#include <algorithm>
#include <cmath>
#include <limits>

void calibration_bench_node::motors_offset_update_callback(
    sl_interfaces::srv::UpdateMotorXYXZOffset_Request::SharedPtr req,
    sl_interfaces::srv::UpdateMotorXYXZOffset_Response::SharedPtr rep) {
  RCLCPP_INFO(get_logger(),
              "recv request: offset_xy:%lf(angle) offset:%lf(angle)",
              req->motor_xy_angle_offset, req->motor_xz_angle_offset);
  motors_handle->update_motors_zero_offset(req->motor_xy_angle_offset,
                                           req->motor_xz_angle_offset);
  rep->set__success(true);
}

void calibration_bench_node::motors_position_control_callback(
    sl_interfaces::srv::MotorXYXZPositionControl_Request::SharedPtr req,
    sl_interfaces::srv::MotorXYXZPositionControl_Response::SharedPtr rep) {
  RCLCPP_INFO(get_logger(),
              "recv request: position_xy:%lf(angle) position_xz:%lf(angle)",
              req->motor_xy_position_angle, req->motor_xz_position_angle);
  motors_handle->motor_xy_sync_position_control(
      PVAD(req->motor_xy_position_angle, 90., 30., 30.));
  motors_handle->motor_xz_sync_position_control(
      PVAD(req->motor_xz_position_angle, 45., 15., 15.));
  rep->success = true;
}
bool calibration_bench_node::init_motors_handle() {
  constexpr frame_id_t FRAME_ID_UNDEFINE = 0;
  const std::string STRING_UNDEFINE = "undefine";
  constexpr double DOUBLE_UNDEFINE = std::numeric_limits<double>::max();
  // TODO: 从parameter获取参数
  this->declare_parameter<std::string>("calibration_bench_name",
                                       STRING_UNDEFINE);
  this->declare_parameter<std::string>("motor_xy_name", STRING_UNDEFINE);
  this->declare_parameter<std::string>("motor_xz_name", STRING_UNDEFINE);
  this->declare_parameter<int>("motor_xy_can_id", FRAME_ID_UNDEFINE);
  this->declare_parameter<int>("motor_xz_can_id", FRAME_ID_UNDEFINE);
  this->declare_parameter<std::string>("can_interface_name", STRING_UNDEFINE);
  this->declare_parameter<double>("motor_xy_offset", DOUBLE_UNDEFINE);
  this->declare_parameter<double>("motor_xz_offset", DOUBLE_UNDEFINE);
  const std::string calibration_bench_name =
      this->get_parameter("calibration_bench_name").as_string();
  const std::string motor_xy_name =
      this->get_parameter("motor_xy_name").as_string();
  const std::string motor_xz_name =
      this->get_parameter("motor_xz_name").as_string();
  const frame_id_t motor_xy_can_id =
      this->get_parameter("motor_xy_can_id").as_int();
  const frame_id_t motor_xz_can_id =
      this->get_parameter("motor_xz_can_id").as_int();
  const std::string can_interface_name =
      this->get_parameter("can_interface_name").as_string();
  const double motor_xy_offset =
      this->get_parameter("motor_xy_offset").as_double();
  const double motor_xz_offset =
      this->get_parameter("motor_xz_offset").as_double();

  // TODO: 检查参数，不能等于初始值
  auto check_undefine = [&]() {
    return (calibration_bench_name != STRING_UNDEFINE) &&
           (motor_xy_name != STRING_UNDEFINE) &&
           (motor_xy_name != STRING_UNDEFINE) &&
           (motor_xy_can_id != FRAME_ID_UNDEFINE) &&
           (motor_xz_can_id != FRAME_ID_UNDEFINE) &&
           (can_interface_name != STRING_UNDEFINE) &&
           (motor_xy_offset != DOUBLE_UNDEFINE) &&
           (motor_xz_offset != DOUBLE_UNDEFINE);
  }();
  if (!check_undefine) {
    RCLCPP_ERROR(get_logger(),
                 "initialization parameters may not use default values");
    exit(-2);
  }

  // TODO: 初始化handle
  calibration_motors_can::init_param_t param{
      .calibration_bench_name = calibration_bench_name,
      .motor_xy_name = motor_xy_name,
      .motor_xz_name = motor_xz_name,
      .motor_xy_can_id = motor_xy_can_id,
      .motor_xz_can_id = motor_xz_can_id,
      .can_device_name = can_interface_name};
  motors_handle = std::make_unique<calibration_motors_can>(param);
  if (!motors_handle->init()) {
    RCLCPP_ERROR(this->get_logger(), "motors_handle init failure");
    return false;
  } else {
    RCLCPP_INFO(this->get_logger(), "motors_handle init success");
    motors_handle->update_motors_zero_offset(motor_xy_offset, motor_xz_offset);
  }
  return true;
}