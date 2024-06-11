#include "sensor_process/sensor_data_record.hpp"
#include <functional>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include <rclcpp/time.hpp>
#include <sl_interfaces/msg/sensor_header.hpp>
#include <sl_interfaces/srv/finish_recorder.hpp>
#include <sl_interfaces/srv/get_sensor_recorder_data.hpp>
#include <sl_interfaces/srv/release_sensor_recorder.hpp>
#include <sl_interfaces/srv/sensor_insert_step_info.hpp>
#include <sl_interfaces/srv/start_new_sensor_recorder.hpp>

class sensor_process_node : public rclcpp::Node {
public:
  sensor_process_node(const std::string &node_name) : rclcpp::Node(node_name) {
    srv_get_sensor_recorder =
        this->create_service<sl_interfaces::srv::GetSensorRecorderData>(
            "get_sensor_recorder_data",
            std::bind(&sensor_process_node::get_sensor_recorder_data_handle,
                      this, std::placeholders::_1, std::placeholders::_2));
    srv_start_new_sensor_recorder =
        this->create_service<sl_interfaces::srv::StartNewSensorRecorder>(
            "start_new_sensor_recorder",
            std::bind(&sensor_process_node::start_new_sensor_recorder_handle,
                      this, std::placeholders::_1, std::placeholders::_2));
    srv_insert_step_info =
        this->create_service<sl_interfaces::srv::SensorInsertStepInfo>(
            "sensor_insert_step_info",
            std::bind(&sensor_process_node::sensor_insert_step_info_handle,
                      this, std::placeholders::_1, std::placeholders::_2));
    srv_release_sensor_recorder =
        this->create_service<sl_interfaces::srv::ReleaseSensorRecorder>(
            "release_sensor_recorder",
            std::bind(&sensor_process_node::release_recorder_handle, this,
                      std::placeholders::_1, std::placeholders::_2));
    srv_finish = this->create_service<sl_interfaces::srv::FinishRecorder>(
        "finish_recorder",
        std::bind(&sensor_process_node::finish_recorder_handle, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  void start_new_sensor_recorder_handle(
      const sl_interfaces::srv::StartNewSensorRecorder::Request::ConstSharedPtr
          request,
      sl_interfaces::srv::StartNewSensorRecorder::Response::SharedPtr response);

  void release_recorder_handle(
      sl_interfaces::srv::ReleaseSensorRecorder::Request::ConstSharedPtr
          request,
      sl_interfaces::srv::ReleaseSensorRecorder::Response::SharedPtr response);

  void get_sensor_recorder_data_handle(
      sl_interfaces::srv::GetSensorRecorderData::Request::ConstSharedPtr
          request,
      sl_interfaces::srv::GetSensorRecorderData::Response::SharedPtr response);

  void sensor_insert_step_info_handle(
      sl_interfaces::srv::SensorInsertStepInfo::Request::ConstSharedPtr request,
      sl_interfaces::srv::SensorInsertStepInfo::Response::SharedPtr response);

  void finish_recorder_handle(
      sl_interfaces::srv::FinishRecorder::Request::ConstSharedPtr request,
      sl_interfaces::srv::FinishRecorder::Response::SharedPtr response);

private:
public:
private:
  std::map<int, std::shared_ptr<sensor_data_recorder>>
      recorders; // <id, handle>
  rclcpp::Service<sl_interfaces::srv::StartNewSensorRecorder>::SharedPtr
      srv_start_new_sensor_recorder;
  rclcpp::Service<sl_interfaces::srv::ReleaseSensorRecorder>::SharedPtr
      srv_release_sensor_recorder;
  rclcpp::Service<sl_interfaces::srv::GetSensorRecorderData>::SharedPtr
      srv_get_sensor_recorder;
  rclcpp::Service<sl_interfaces::srv::SensorInsertStepInfo>::SharedPtr
      srv_insert_step_info;
  rclcpp::Service<sl_interfaces::srv::FinishRecorder>::SharedPtr srv_finish;
};
