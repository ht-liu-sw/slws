/**
 * @file calibration_motors.hpp
 * @author 刘健成 (liujiancheng@haptron.com)
 * @brief 标定用的电机控制API
 * @version 0.1
 * @date 2024-05-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <algorithm>
#include <calibration_motors_can.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <sl_interfaces/srv/update_motor_xyxz_offset.hpp>
#include <sl_interfaces/srv/motor_xyxz_position_control.hpp>

class calibration_bench_node : public rclcpp::Node {
public:
  calibration_bench_node() : rclcpp::Node("calibration_bench") {
    
    init_motors_handle();
  }

  void motors_offset_update_callback(
    sl_interfaces::srv::UpdateMotorXYXZOffset_Request::SharedPtr req,
    sl_interfaces::srv::UpdateMotorXYXZOffset_Response::SharedPtr rep 
  );

  void motors_position_control_callback(
    sl_interfaces::srv::MotorXYXZPositionControl_Request::SharedPtr req, 
    sl_interfaces::srv::MotorXYXZPositionControl_Response::SharedPtr rep);
private:
  bool init_motors_handle();
    
private:
  std::atomic<bool> inited_offset{false};
  std::unique_ptr<calibration_motors_can> motors_handle;
  rclcpp::Service<sl_interfaces::srv::MotorXYXZPositionControl>::SharedPtr position_control_service;
};
