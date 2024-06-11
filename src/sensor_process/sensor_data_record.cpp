#include "sensor_process/sensor_data_record.hpp"
#include <cstdio>

#include "sensor_conn/sensor_conn_modbus_tcp.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <sl_interfaces/msg/sensor_record_single_frame.hpp>

auto sensor_data_recorder::connect() -> bool {

  // TODO: 判断 buffer.header.sensor_type_tag 是否为 MODBUS_RTU 连接
  if (CONNECT_TYPE::MODBUS_TCP == CONNECT_TYPE::MODBUS_TCP) {
    const sensor_conn_modbus_tcp::connect_params_t conn_params{
        .remote_ip = info.header.modbus_tcp_ip,
        .remote_port = info.header.modbus_tcp_port,
        .slave_id = info.header.modbus_tcp_slave_id,
        .address_map_str = info.header.address_map};
    handle = std::make_unique<sensor_conn_modbus_tcp>(conn_params);
    const auto handle_init_ret = handle->init();
    if (handle_init_ret == false) {
      RCLCPP_WARN(this->get_logger(), "sensor_conn_modbus_tcp init failure");
      return false;
    }
    return true;
  }
}

auto sensor_data_recorder::start_record(const rclcpp::Time stamp) -> bool {
  info.recored_data.start_time = stamp;
  need_cap_flag = true;
  RCLCPP_INFO(this->get_logger(), "start_record");
  if (cap_thread == nullptr) {
    // TODO: 开启线程，持续获取数据
    cap_thread = std::make_unique<std::thread>([this] {
      while (need_cap_flag) {
        // 获取数据和相应时间戳
        auto cap_data_and_time = [this]() {
          const auto before_time = rclcpp::Clock().now();
          const auto &&cap_data = handle->capture_once();
          const auto after_time = rclcpp::Clock().now();
          const auto cap_time =
              rclcpp::Time((after_time - before_time).nanoseconds() / 2 +
                           before_time.nanoseconds());
          return std::tuple(cap_time, cap_data);
        };
        auto &&[cap_time, cap_data] = cap_data_and_time();
        if (cap_data.has_value()) {
          sl_interfaces::msg::SensorRecordSingleFrame frame;
          frame.set__timestamp(cap_time);
          frame.set__ordered_data(std::vector<double>(cap_data.value().cbegin(),
                                                      cap_data.value().cend()));
          this->info.recored_data.data_frames.push_back(frame);
        } else {
          // TODO: 错误信息，收不到数据
          RCLCPP_WARN(get_logger(), "recv sensor data failure");
        }
        // each time wait 100us
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });
    return true;
  } else {
    // TODO: 通知线程已经开启
    RCLCPP_WARN(get_logger(), "record thread already start");
    return false;
  }
}

auto sensor_data_recorder::finish_record(const rclcpp::Time stamp) -> bool {
  info.recored_data.finish_time = stamp;
  if (cap_thread->joinable()) {
    need_cap_flag = false;
    cap_thread->join();
    RCLCPP_INFO(this->get_logger(), "record finish, thread stopped");
    RCLCPP_INFO(this->get_logger(), "record[%d] recved %ld frame",
                info.recored_data.id, info.recored_data.data_frames.size());
    return true;
  } else {
    RCLCPP_WARN(this->get_logger(), "线程已提前退出");
  }
  return true;
}

auto sensor_data_recorder::get_id() -> int { return info.recored_data.id; }

auto sensor_data_recorder::step_record(
    const sl_interfaces::msg::SensorRecordStepInfo &step_info) -> bool {
  const auto start_time = rclcpp::Time(step_info.start_time);
  const auto finish_time = rclcpp::Time(step_info.finish_time);
  RCLCPP_INFO(this->get_logger(),
              "insert step_info: tag:%s, start_time(s):%lf, stop_time(s):%lf",
              step_info.tag.c_str(), start_time.seconds(),
              finish_time.seconds());
  info.recored_data.step_info.push_back(step_info);
  return true;
}

auto sensor_data_recorder::get_data() -> sl_interfaces::msg::SensorInfo {
  return info;
}