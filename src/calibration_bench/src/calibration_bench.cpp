#include "calibration_bench.hpp"
#include "motor_template.h"

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
      V90_A30_D30(req->motor_xy_position_angle));
  motors_handle->motor_xz_sync_position_control(
      V45_A15_D15(req->motor_xz_position_angle));
  rep->success = true;
}
bool calibration_bench_node::init_motors_handle() {
  constexpr unsigned int UINT_UNDEFINE = -1;
  constexpr frame_id_t FRAME_ID_UNDEFINE = 0;
  const std::string STRING_UNDEFINE = "undefine";
  // TODO: 从parameter获取参数
  this->declare_parameter<std::string>("calibration_bench_name",
                                       STRING_UNDEFINE);
  this->declare_parameter<std::string>("motor_xy_name", STRING_UNDEFINE);
  this->declare_parameter<std::string>("motor_xz_name", STRING_UNDEFINE);
  this->declare_parameter<int>("DeviceInd", UINT_UNDEFINE);
  this->declare_parameter<int>("CANChannel", UINT_UNDEFINE);
  this->declare_parameter<int>("motor_xy_frame_id", FRAME_ID_UNDEFINE);
  this->declare_parameter<int>("motor_xz_frame_id", FRAME_ID_UNDEFINE);
  const std::string calibration_bench_name =
      this->get_parameter("calibration_bench_name").as_string();
  const std::string motor_xy_name =
      this->get_parameter("motor_xy_name").as_string();
  const std::string motor_xz_name =
      this->get_parameter("motor_xz_name").as_string();
  constexpr auto DeviceType = USBCAN1;
  const unsigned int DeviceInd = this->get_parameter("DeviceInd").as_int();
  const unsigned int CANChannel = this->get_parameter("CANChannel").as_int();
  const frame_id_t motor_xy_frame_id =
      this->get_parameter("motor_xy_frame_id").as_int();
  const frame_id_t motor_xz_frame_id =
      this->get_parameter("motor_xz_frame_id").as_int();

  // TODO: 检查参数，不能等于初始值
  auto check_undefine = [&]() {
    return (calibration_bench_name != STRING_UNDEFINE) &&
           (motor_xy_name != STRING_UNDEFINE) &&
           (motor_xy_name != STRING_UNDEFINE) && (DeviceInd != UINT_UNDEFINE) &&
           (CANChannel != UINT_UNDEFINE) &&
           (motor_xy_frame_id != FRAME_ID_UNDEFINE) &&
           (motor_xz_frame_id != FRAME_ID_UNDEFINE);
  }();
  if (check_undefine == false) {
    RCLCPP_ERROR(get_logger(),
                 "initialization parameters may not use default values");
    return false;
  }

  // TODO: 初始化handle
  can_info_t can_info{.DeviceType = DeviceType,
                      .DeviceInd = DeviceInd,
                      .CANChannel = CANChannel};
  calibration_motors_can::init_param_t param{
      .calibration_bench_name = calibration_bench_name,
      .motor_xy_name = motor_xy_name,
      .motor_xz_name = motor_xz_name,
      .can_info = can_info,
      .motor_xy_can_id = motor_xy_frame_id,
      .motor_xz_can_id = motor_xz_frame_id};
  motors_handle = std::make_unique<calibration_motors_can>(param);
  if (!motors_handle->init_can_device()) {
    RCLCPP_ERROR(this->get_logger(), "init_can_device failure");
    return false;
  } else {
    RCLCPP_INFO(this->get_logger(), "init_can_device success");
  }
  if (!motors_handle->init_motors_handle()) {
    RCLCPP_ERROR(this->get_logger(), "init_motors_handle failure");
    return false;
  } else {
    RCLCPP_INFO(this->get_logger(), "init_motors_handle success");
  }
  return true;
}