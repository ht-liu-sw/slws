#include <sensor_process_node.hpp>

void sensor_process_node::start_new_sensor_recorder_handle(
    const sl_interfaces::srv::StartNewSensorRecorder::Request::ConstSharedPtr
        request,
    sl_interfaces::srv::StartNewSensorRecorder::Response::SharedPtr response) {
  const auto init_info = request->sensor_recorder_init_info;
  RCLCPP_INFO(
      this->get_logger(),
      "start new recorder ,info:\ntype: %s\ntag: %s\nmodbus_tcp_ip: "
      "%s\nmodbus_tcp_port: %d\nmodbus_tcp_slave_id: %d\naddress_map: %s\n ",
      init_info.sensor_type_tag.c_str(), init_info.tag.c_str(),
      init_info.modbus_tcp_ip.c_str(), init_info.modbus_tcp_port,
      init_info.modbus_tcp_slave_id, init_info.address_map.c_str());
  auto recorder = std::make_shared<sensor_data_recorder>(init_info);
  auto connect_ret = recorder->connect();
  if(connect_ret == true){
    RCLCPP_INFO(this->get_logger(), "recorder connect success");
  }else{
    RCLCPP_WARN(this->get_logger(), "recorder connect failure");
  }
  rclcpp::Time start_time = rclcpp::Clock().now();
  const auto ret = recorder->start_record(start_time);
  if (ret == false) {
    RCLCPP_WARN(this->get_logger(), "start_record failure");
    response->set__success(false);
    return;
  } else {
    RCLCPP_INFO(this->get_logger(), "start sensor recorder[%d] finish",
                recorder->get_id());
    recorders[recorder->get_id()] = recorder;
    response->set__success(true);
    response->new_recorder_data_id = recorder->get_id();
  }
}

void sensor_process_node::release_recorder_handle(
    sl_interfaces::srv::ReleaseSensorRecorder::Request::ConstSharedPtr request,
    sl_interfaces::srv::ReleaseSensorRecorder::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "%s need release recorder[%d]", __func__,
              request->id);
  if (recorders.find(request->id) != recorders.cend()) {
    recorders.erase(recorders.find(request->id));
    RCLCPP_INFO(this->get_logger(), "release recorder[%d] success",
                request->id);
    response->set__success(true);
  } else {
    RCLCPP_WARN(this->get_logger(), "recorder[%d] not found", request->id);
    response->set__success(false);
    response->set__message("recoder id [" + std::to_string(request->id) +
                           "] not found");
  }
  return;
}

void sensor_process_node::get_sensor_recorder_data_handle(
    sl_interfaces::srv::GetSensorRecorderData::Request::ConstSharedPtr request,
    sl_interfaces::srv::GetSensorRecorderData::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "%s need finish recorder[%d]", __func__,
              request->id);
  if (recorders.find(request->id) != recorders.cend()) {
    auto &&info = recorders.at(request->id)->get_data();
    RCLCPP_INFO(this->get_logger(), "%s [%d] success", __func__, request->id);
    response->set__success(true);
    auto header = info.header;
    response->data.set__header(header);
    response->data.set__recored_data(info.recored_data);

  } else {
    RCLCPP_WARN(this->get_logger(), "recorder[%d] not found", request->id);
    response->set__success(false);
    response->set__message("recoder id [" + std::to_string(request->id) +
                           "] not found");
  }
  return;
}

void sensor_process_node::finish_recorder_handle(
    sl_interfaces::srv::FinishRecorder::Request::ConstSharedPtr request,
    sl_interfaces::srv::FinishRecorder::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "%s need finish recorder[%d]", __func__,
              request->id);
  if (recorders.find(request->id) != recorders.cend()) {
    const auto ret =
        recorders.at(request->id)->finish_record(rclcpp::Clock().now());
    if (ret == true) {
      RCLCPP_INFO(this->get_logger(), "%s [%d] success", __func__, request->id);
      response->set__success(true);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s finish recorder failure", __func__);
      response->set__success(false);
      response->set__message("recoder id [" + std::to_string(request->id) +
                             "] found but finish failure");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "recorder[%d] not found", request->id);
    response->set__success(false);
    response->set__message("recoder id [" + std::to_string(request->id) +
                           "] not found");
  }
  return;
}

void sensor_process_node::sensor_insert_step_info_handle(
    sl_interfaces::srv::SensorInsertStepInfo::Request::ConstSharedPtr request,
    sl_interfaces::srv::SensorInsertStepInfo::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "%s in recorder[%d]", __func__,
              request->recorder_id);
  if (recorders.find(request->recorder_id) != recorders.cend()) {

    const auto ret =
        recorders.at(request->recorder_id)->step_record(request->step_info);
    if (ret == true) {
      RCLCPP_INFO(this->get_logger(), "%s [%d] success", __func__,
                  request->recorder_id);
      response->set__success(true);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s finish recorder failure", __func__);
      response->set__success(false);
      response->set__message("recoder id [" +
                             std::to_string(request->recorder_id) +
                             "] found but finish failure");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "recorder[%d] not found",
                request->recorder_id);
    response->set__success(false);
    response->set__message(
        "recoder id [" + std::to_string(request->recorder_id) + "] not found");
  }
  return;
}