#pragma once

#include "sensor_conn/sensor_conn.hpp"
#include "sensor_conn/sensor_conn_modbus_tcp.hpp"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <sl_interfaces/msg/detail/sensor_info__struct.hpp>
#include <sl_interfaces/msg/detail/sensor_record_step_info__struct.hpp>
#include <sl_interfaces/msg/sensor_header.hpp>
#include <sl_interfaces/msg/sensor_info.hpp>
#include <string>
#include <thread>

enum class CONNECT_TYPE { MODBUS_TCP = 1 };

/**
 * @brief 内部初始化传感器通讯，调用start_capture后创建线程循环记录
 *
 */
class sensor_data_recorder {
public:
  sensor_data_recorder(const sl_interfaces::msg::SensorHeader &init_param) {
    static int id = 0;
    ++id;
    // 存储初始化数据
    info.set__header(init_param);
    // 初始化id
    info.recored_data.id = id;
    // 初始化日志器参数
    logger_name = "sensor_data_recorder[" + info.header.tag + "|" +
                  info.header.sensor_type_tag + "|" + std::to_string(id) + "]";
  };
  auto connect() -> bool;
  auto start_record(const rclcpp::Time stamp) -> bool;
  auto finish_record(const rclcpp::Time stamp) -> bool;
  auto step_record(const sl_interfaces::msg::SensorRecordStepInfo& step_info) -> bool;
  auto get_id() -> int;
  auto get_data() -> sl_interfaces::msg::SensorInfo;

private:
  auto capture_data_once() -> std::vector<uint16_t>;

  inline auto get_logger() -> rclcpp::Logger {
    return rclcpp::get_logger(logger_name);
  }

private:
  sl_interfaces::msg::SensorInfo info;
  std::unique_ptr<sensor_conn_modbus_tcp> handle = nullptr;
  std::unique_ptr<std::thread> cap_thread = nullptr;
  std::atomic<bool> need_cap_flag;
  std::string logger_name;
};
