#include "motor_template.h"
#include "zeroerr_motor_can.h"
#include <calibration_motors_can.hpp>

int main() {
  calibration_motors_can::init_param_t init_param{
      .calibration_bench_name = "motors",
      .motor_xy_name = "motor_xy",
      .motor_xz_name = "motor_xz",
      .can_info =
          can_info_t{.DeviceType = USBCAN1, .DeviceInd = 0, .CANChannel = 0},
      .motor_xy_can_id = 0x0B,
      .motor_xz_can_id = 0x03};
  auto handle = calibration_motors_can(init_param);
  std::cout << "init can device: " << handle.init_can_device() << std::endl;
  std::cout << "init motors_handle: " << handle.init_motors_handle()
            << std::endl;
  handle.update_motors_zero_offset(0.0, 0.0);
  while (true) {
    auto poss = handle.get_motors_position();
    if (poss.has_value()) {
      std::cout << "<position_xy, position_xz>: "
                << poss.value().motor_xy_source_angle << ", "
                << poss.value().motor_xz_offseted_angle << std::endl;
      // handle.motor_xy_sync_position_control(V90_A30_D30(180.));
      // handle.motor_xz_sync_position_control(V45_A15_D15(180.));
      // handle.motor_xz_homing();
      // handle.motor_xy_homing();
    }
  }
}