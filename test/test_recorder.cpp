#include "modbus-tcp.h"
#include "modbus.h"
#include "sensor_conn/sensor_conn_modbus_tcp.hpp"

#include <iostream>
void sensor_conn() {
  sensor_conn_modbus_tcp::sensor_conn_modbus_tcp_connect_parameter conn_param{
      .remote_ip = "172.16.1.120",
      .remote_port = 39393,
      .slave_id = 0x01,
      .address_map_str = R"({  
               "ordered_map": [          
                 {
                   "name": "adc_original",   
                   "channel_num": 16,         
                   "type": "float"           
                 },
                 {
                   "name": "temp",           
                   "channel_num": 4,
                   "type": "float"
                 },
                 {
                   "name": "adc_processed", 
                   "channel_num": 8,
                   "type": "float"
                 },
                 {
                   "name": "force",
                   "channel_num": 3,
                   "type": "float"
                 },
                 {
                   "name": "torque",
                   "channel_num": 3,
                   "type": "float"
                 }
               ]
             })"};
  sensor_conn_modbus_tcp handle(conn_param);
  auto init_ret = handle.init();
  std::cout << "init " << init_ret << std::endl;
  while (true) {
    auto data = handle.capture_once();
    std::cout << "--------" << std::endl;
    if (data.has_value()) {
      const auto &vf = data.value();
      for (auto &item : vf) {
        std::cout << item << std::endl;
      }
    }
  }
}

void test_modbus() {
  auto mb = modbus_new_tcp("172.16.1.120", 39393);
  modbus_connect(mb);
  modbus_set_slave(mb, 1);
  std::array<uint16_t, 10> arr;
  auto ret = modbus_read_input_registers(mb, 0x0000, 10, arr.data());
  printf("%d\n", ret);
}

int main(const int, const char **) { sensor_conn(); }