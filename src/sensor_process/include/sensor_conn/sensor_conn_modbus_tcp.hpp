#pragma once

#include <modbus-tcp.h>
#include "sensor_conn.hpp"
#include "sensor_conn/misc.hpp"
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <modbus.h>
#include <optional>
#include <rclcpp/logger.hpp>
#include <string>

class sensor_conn_modbus_tcp final : public sensor_conn {

public:
  using connect_params_t = struct sensor_conn_modbus_tcp_connect_parameter {
    std::string remote_ip;
    uint32_t remote_port;
    uint8_t slave_id;
    std::string address_map_str;
  };

public:
  sensor_conn_modbus_tcp(const connect_params_t &conn_param)
      : params(conn_param){};
  ~sensor_conn_modbus_tcp() { exit(); }

  auto init() -> bool override;
  bool exit() override;
  auto capture_once() -> std::optional<std::vector<double>> override;

private:
  auto init_capture_map() -> bool;

private:
  bool inited = false;
  bool connected = false;
  modbus_t *mb = nullptr;
  const connect_params_t params;
  std::shared_ptr<address_mapper> mapper_ptr = nullptr;
};
