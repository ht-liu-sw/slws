#include "sensor_conn/sensor_conn_modbus_tcp.hpp"
#include "modbus.h"
#include <cstdio>
#include <iostream>

auto sensor_conn_modbus_tcp::init_capture_map() -> bool {
  try {
    mapper_ptr = std::make_shared<address_mapper>(params.address_map_str);
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    mapper_ptr = nullptr;
    return false;
  }
  return true;
}

auto sensor_conn_modbus_tcp::init() -> bool {
  auto connect = [this]() {
    if (mb != nullptr) {
      modbus_set_response_timeout(mb, 1, 0);
      auto con_ret = modbus_connect(mb);
      if (con_ret == 0 && connected == false) {
        connected = true;
        modbus_set_slave(mb, params.slave_id);
        return true;
      }
    }
    // 连接条件不满足
    connected = false;
    return false;
  };

  std::vector<char> ip_str_buffer(params.remote_ip.size() + 1, 0);
  memcpy(ip_str_buffer.data(), params.remote_ip.data(),
         params.remote_ip.size());
  mb = modbus_new_tcp(params.remote_ip.data(), params.remote_port);
  if (mb != nullptr) {
    // 链接成功
    auto ret = (connect() && this->init_capture_map());
    inited = ret;
    return ret;
  }
  inited = false;
  return false;
}



auto sensor_conn_modbus_tcp::capture_once()
    -> std::optional<std::vector<double>> {
  if (mapper_ptr == nullptr) {
    // TODO: 通知无映射数据
    return std::nullopt;
  }
  const auto frame_size_uint16 = mapper_ptr->get_frame_size_uint16();
  auto original_data = std::vector<uint16_t>(frame_size_uint16, 0);
  const auto ret = modbus_read_input_registers(mb, 0x0000, frame_size_uint16,
                                               original_data.data());
  if (ret == -1) {
    auto error_msg = std::string(strerror(errno));
    perror("modbus recv");
    return std::nullopt;
  }

  const auto frame_size_float = mapper_ptr->get_frame_size_float32();
  auto result = std::vector<double>(0,0);
  // 处理数据
  for (size_t i = 0; i < frame_size_float; ++i) {
    using std::swap;
    swap<uint16_t>(original_data.at(0 + i * 2), original_data.at(1 + i * 2));
  }
  // 拷贝数据(注意类型差异!!!)
  auto fp = reinterpret_cast<const float *>(original_data.data());
  for(size_t i = 0; i < mapper_ptr->get_frame_size_float32(); ++i){
    result.push_back(fp[i]);
  }
  return {std::move(result)};
}

auto sensor_conn_modbus_tcp::exit() -> bool {
  if (mb != nullptr) {
    if (connected) {
      modbus_close(mb);
    }
    modbus_free(mb);
    mb = nullptr;
  }
  return true;
}