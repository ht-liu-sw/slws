#include <chrono>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sl_interfaces/srv/detail/finish_recorder__struct.hpp>
#include <sl_interfaces/srv/detail/get_sensor_recorder_data__struct.hpp>
#include <sl_interfaces/srv/detail/release_sensor_recorder__struct.hpp>
#include <sl_interfaces/srv/detail/sensor_insert_step_info__struct.hpp>
#include <sl_interfaces/srv/detail/start_new_sensor_recorder__struct.hpp>
#include <sl_interfaces/srv/finish_recorder.hpp>
#include <sl_interfaces/srv/get_sensor_recorder_data.hpp>
#include <sl_interfaces/srv/release_sensor_recorder.hpp>
#include <sl_interfaces/srv/sensor_insert_step_info.hpp>
#include <sl_interfaces/srv/start_new_sensor_recorder.hpp>
#include <thread>

using namespace sl_interfaces::srv;
class test_node : public rclcpp::Node {
public:
  test_node(std::string name) : rclcpp::Node(name){};

public:
  rclcpp::Client<sl_interfaces::srv::StartNewSensorRecorder>::SharedPtr
      cli_new_start;
  rclcpp::Client<sl_interfaces::srv::FinishRecorder>::SharedPtr cli_finish;
  rclcpp::Client<sl_interfaces::srv::GetSensorRecorderData>::SharedPtr
      cli_getdata;
  rclcpp::Client<sl_interfaces::srv::ReleaseSensorRecorder>::SharedPtr
      cli_release;
  rclcpp::Client<sl_interfaces::srv::SensorInsertStepInfo>::SharedPtr cli_step;
  rclcpp::TimerBase::SharedPtr test_timer;
};

void test(rclcpp::Node::SharedPtr node) {

  auto cli_new_start =
      node->create_client<StartNewSensorRecorder>("start_new_sensor_recorder");
  auto cli_finish = node->create_client<FinishRecorder>("finish_recorder");
  auto cli_getdata =
      node->create_client<GetSensorRecorderData>("get_sensor_recorder_data");
  auto cli_step =
      node->create_client<SensorInsertStepInfo>("sensor_insert_step_info");
  auto cli_release =
      node->create_client<ReleaseSensorRecorder>("release_sensor_recorder");

  // start
  StartNewSensorRecorder::Request::SharedPtr start_req =
      std::make_shared<StartNewSensorRecorder::Request>();
  start_req->sensor_recorder_init_info.set__sensor_type_tag("test_type")
      .set__modbus_tcp_ip("172.16.1.120")
      .set__modbus_tcp_port(39393)
      .set__modbus_tcp_slave_id(0x01)
      .set__address_map(R"({  
               "ordered_map": [          
                 {
                   "name": "adc_original",   
                   "channel_num": 8,         
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
             })")
      .set__tag("tag");
  auto start_rep = cli_new_start->async_send_request(start_req);
  int id = 0;
  if (rclcpp::spin_until_future_complete(node, start_rep) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto value = start_rep.get();
    id = value->new_recorder_data_id;
    RCLCPP_INFO(rclcpp::get_logger("test"), "start success, id = %d", id);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("test"), "start failure");
  }
  RCLCPP_INFO(rclcpp::get_logger("test"), "wait 3s");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // step1
  SensorInsertStepInfo_Request::SharedPtr step_req =
      std::make_shared<SensorInsertStepInfo_Request>();
  step_req->set__recorder_id(id)
      .step_info.set__tag("step1")
      .set__start_time(rclcpp::Clock().now())
      .set__finish_time(rclcpp::Clock().now());
  auto step_rep = cli_step->async_send_request(step_req);

  if (rclcpp::spin_until_future_complete(node, step_rep) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = step_rep.get();
    RCLCPP_INFO(rclcpp::get_logger("test"), "step success");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("test"), "step failure");
  }

  RCLCPP_INFO(rclcpp::get_logger("test"), "wait 3s");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // step2
  step_req->set__recorder_id(id)
      .step_info.set__tag("step2")
      .set__start_time(rclcpp::Clock().now())
      .set__finish_time(rclcpp::Clock().now());
  step_rep = cli_step->async_send_request(step_req);
  if (rclcpp::spin_until_future_complete(node, step_rep) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = step_rep.get();
    RCLCPP_INFO(rclcpp::get_logger("test"), "step success");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("test"), "step failure");
  }

  RCLCPP_INFO(rclcpp::get_logger("test"), "wait 3s");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // step3
  step_req->set__recorder_id(id)
      .step_info.set__tag("step3")
      .set__start_time(rclcpp::Clock().now())
      .set__finish_time(rclcpp::Clock().now());
  step_rep = cli_step->async_send_request(step_req);
  if (rclcpp::spin_until_future_complete(node, step_rep) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = step_rep.get();
    RCLCPP_INFO(rclcpp::get_logger("test"), "step success");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("test"), "step failure");
  }


  // finish
  FinishRecorder_Request::SharedPtr fin_req =
      std::make_shared<FinishRecorder_Request>();
  fin_req->set__id(id);
  auto fin_rep = cli_finish->async_send_request(fin_req);
  if (rclcpp::spin_until_future_complete(node, fin_rep) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = fin_rep.get();
    RCLCPP_INFO(rclcpp::get_logger("test"), "finish success");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("test"), "finish failure");
  }


  // get_data
  GetSensorRecorderData_Request::SharedPtr get_req =
      std::make_shared<GetSensorRecorderData_Request>();
  get_req->set__id(id);
  auto get_rep = cli_getdata->async_send_request(get_req);
  if (rclcpp::spin_until_future_complete(node, get_rep) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = get_rep.get();
    RCLCPP_INFO(rclcpp::get_logger("test"), "step success");
    RCLCPP_INFO(rclcpp::get_logger("test"), "get success");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("test"), "get failure");
  }
  RCLCPP_INFO(rclcpp::get_logger("test"), "wait 3s");
  std::this_thread::sleep_for(std::chrono::seconds(3));

//   // release
//   ReleaseSensorRecorder_Request::SharedPtr rel_req =
//       std::make_shared<ReleaseSensorRecorder_Request>();
//   rel_req->set__id(id);
//   auto rel_rep = cli_release->async_send_request(rel_req);
//   if (rclcpp::spin_until_future_complete(node, rel_rep) ==
//       rclcpp::FutureReturnCode::SUCCESS) {
//     auto result = rel_rep.get();
//     RCLCPP_INFO(rclcpp::get_logger("test"), "release success");
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("test"), "release failure");
//   }
//   RCLCPP_INFO(rclcpp::get_logger("test"), "wait 3s");
//   std::this_thread::sleep_for(std::chrono::seconds(3));
}

int main(const int argc, const char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<test_node>("test_node");
  test(node);
  //rclcpp::spin(node);
  rclcpp::shutdown();
}