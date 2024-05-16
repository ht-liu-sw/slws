#include "calibration_bench.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, const char*const * argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<calibration_bench_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}