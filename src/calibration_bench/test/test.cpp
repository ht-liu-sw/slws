//#include "motor_template.h"
#include "motor_template.h"
#include <calibration_motors_can.hpp>
#include <chrono>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  calibration_motors_can::init_param_t init_param{.calibration_bench_name =
                                                      "motors",
                                                  .motor_xy_name = "motor_xy",
                                                  .motor_xz_name = "motor_xz",
                                                  .motor_xy_can_id = 0x0B,
                                                  .motor_xz_can_id = 0x03,
                                                  .can_device_name = "can0"};
  auto handle = calibration_motors_can(init_param);
  std::cout << "init can device: " << handle.init() << std::endl;
  handle.update_motors_zero_offset(0.0, 0.0);
  auto logger = rclcpp::get_logger(__func__);
  while (true) {
    auto poss = handle.get_motors_position();
    if (poss.has_value()) {
      RCLCPP_INFO(logger, "<position_xy, position_xz>: %lf, %lf",
                  poss.value().motor_xy_source_angle,
                  poss.value().motor_xz_source_angle);
      handle.motor_xy_sync_position_control(PVAD(180., 15., 5., 5.));
      handle.motor_xz_sync_position_control(PVAD(180., 15., 5., 5.));
      handle.motor_xz_homing(VAD(15., 5., 5.));
      handle.motor_xy_homing(VAD(15., 5., 5.));
      // handle.motor_xy_activate_break();
      // std::this_thread::sleep_for(std::chrono::milliseconds(1'000));
      // handle.motor_xy_release_break();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1'000));
  }
  rclcpp::shutdown();
}