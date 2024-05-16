#include "zeroerr_motor_can.h"
#include "ECanVci.h"
#include "motor_template.h"
#include "zeroerr_motor_can.h"
#include "zeroerr_motor_struct_can.hpp"
#include <bit>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <thread>

namespace {
using std::this_thread::sleep_for;
}

bool zeroerr_motor_can::position_control(
    const motor_position_control_parameter &param, bool sync_motion) {
  RCLCPP_INFO(logger, "%s target_position(angle): %lf, sync_motion: %s",
              __func__, param.target_position_angle,
              (sync_motion ? "true" : "false"));
  auto angle2coder = [](const double angle) {
    constexpr int32_t coder_cnt = (1l << 19);
    return angle / 360.0 * coder_cnt;
  };

  [this, angle2coder](const motor_position_control_parameter &param) {
    // 设置参数
    const auto target_position_angle = param.target_position_angle;
    auto cp = dump_zeroerr_communication_parameter(
        angle2coder(target_position_angle),
        angle2coder(param.profile_velocity_angle),
        angle2coder(param.profile_acceleration_angle),
        angle2coder(param.profile_deceleration_angle));

    // write_parameters(cp);
    auto write_single_warp = [this](const motor_item_can &item,
                                    const std::string &part_msg) -> bool {
      // write_single
      const auto ret = this->local_transmit(item);
      this->local_recv();

      if (ret)
        RCLCPP_INFO(logger, "%s", (part_msg + " success").c_str());
      else
        RCLCPP_INFO(logger, "%s", (part_msg + " failure").c_str());
      return ret;
    };
    write_single_warp(cp->motion_mode, "set motion_mode");
    write_single_warp(cp->profile_velocity, "set profile_velocity");
    write_single_warp(cp->profile_acceleration, "set profile_acceleration");
    write_single_warp(cp->profile_deceleration, "set profile_deceleration");
    write_single_warp(cp->target_position, "set target_position");

    motor_enable();
    // 开始运动
    motion_start();
  }(param);
  if (sync_motion) {
    auto motion_ret = wait_motion_finish();
    return motion_ret;
  }
  return true;
}

std::shared_ptr<zeroerr_communication_parameter>
zeroerr_motor_can::dump_zeroerr_communication_parameter(
    const int32_t target_pose, const int32_t profile_velocity,
    const int32_t profile_acceleration, const int32_t profile_deceleration) {
  // static_assert(expression, message);
  auto split_int32_to = [](int32_t val, std::vector<uint8_t> &vec) {
    auto ptr = reinterpret_cast<uint8_t *>(&val);
    vec.pop_back();
    vec.pop_back();
    vec.pop_back();
    vec.pop_back();
    if (std::endian::native == std::endian::big) {
      vec.push_back(ptr[0]);
      vec.push_back(ptr[1]);
      vec.push_back(ptr[2]);
      vec.push_back(ptr[3]);
    } else if (std::endian::native == std::endian::little) {
      vec.push_back(ptr[3]);
      vec.push_back(ptr[2]);
      vec.push_back(ptr[1]);
      vec.push_back(ptr[0]);
    }
  };

  auto result = std::make_shared<zeroerr_communication_parameter>();
  {
    split_int32_to(target_pose, result->target_position);
    split_int32_to(profile_velocity, result->profile_velocity);
    split_int32_to(profile_acceleration, result->profile_acceleration);
    split_int32_to(profile_deceleration, result->profile_deceleration); /*
 qDebug() << result->control_mode.data;
 qDebug() << (motor_data_t)result->motion_mode.data;
 qDebug() << result->target_position.data;
 qDebug() << result->profile_velocity.data;
 qDebug() << result->profile_acceleration.data;
 qDebug() << result->profile_deceleration.data;*/
  }
  return result;
}

o_angle_t zeroerr_motor_can::read_angle() {
  //// 清除缓存
  // ClearBuffer(can_info.DeviceType, can_info.DeviceInd, can_info.CANChannel);
  zeroerr_read_angle tpara{};
  const auto ret_transmit = this->local_transmit(tpara.data);
  RCLCPP_INFO(logger, "transmit reture: %d", ret_transmit);
  auto recv_data = this->local_recv();
  if (recv_data.has_value()) {
    if (!recv_data->empty()) {
      constexpr auto ZEROERR_CAN_TAIL_NORMAL = 0x3E;
      constexpr auto ZEROERR_CAN_TAIL_ERROR = 0x80;
      if (recv_data->back() == ZEROERR_CAN_TAIL_NORMAL) {
        // zeroerr 正常结束
        int32_t res = 0;
        res |= recv_data->at(0);
        res <<= 8;
        res |= recv_data->at(1);
        res <<= 8;
        res |= recv_data->at(2);
        res <<= 8;
        res |= recv_data->at(3);

        constexpr int32_t coder_cnt = (1 << 19);
        const double original_angle = (((double)res) / coder_cnt) * 360.0;
        return {original_angle};
      } else if(recv_data->back() == ZEROERR_CAN_TAIL_ERROR){
        RCLCPP_WARN(logger, "收到异常");
        return std::nullopt;
      } else{
        RCLCPP_ERROR(logger, "严重错误，接收到未知数据末尾: 0x%x", recv_data->back());
        return std::nullopt;
      }
    } else {
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }
}

// 刹车动作安全时间
constexpr auto SAFE_MOTOR_BREAK_WAIT_TIME = std::chrono::milliseconds(200);
bool zeroerr_motor_can::motor_enable() {
  RCLCPP_DEBUG(logger, __func__);
  zeroerr_enable_motor_parameter tpara;
  const auto ret = this->local_transmit(tpara.data);
  // 电机使能后等待安全时间，刹车动作时间约150ms, 取200ms
  if (ret == true) {
    sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
    return true;
  }
  return false;
}
bool zeroerr_motor_can::motor_disable() {
  RCLCPP_DEBUG(logger, __func__);
  zeroerr_disable_motor_parameter tpara;
  const auto ret = this->local_transmit(tpara.data);
  // 电机停止后等待安全时间，刹车动作时间约150ms
  if (ret == true) {
    sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
    return true;
  }
  return false;
}

bool zeroerr_motor_can::motion_start() {
  RCLCPP_DEBUG(logger, __func__);
  zeroerr_start_motion_parameter tpara;
  const auto ret = this->local_transmit(tpara.data);
  // 电机停止后等待安全时间，刹车动作时间约150ms
  if (ret == true) {
    sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
    return true;
  }
  return false;
}
bool zeroerr_motor_can::motion_stop() {
  RCLCPP_DEBUG(logger, __func__);
  zeroerr_stop_motion_parameter tpara;
  const auto ret = this->local_transmit(tpara.data);
  // 电机停止后等待安全时间，刹车动作时间约150ms
  if (ret == true) {
    sleep_for(SAFE_MOTOR_BREAK_WAIT_TIME);
    return true;
  }
  return false;
}

bool zeroerr_motor_can::wait_motion_finish() {
  RCLCPP_DEBUG(logger, "motion start");
  // 等待运动结束
  auto motion_ret = [this]() -> bool {
    auto motion_finish = false;
    do {
      zeroerr_read_target_reach_status tpara;
      ClearBuffer(can_info.DeviceType, can_info.DeviceInd, can_info.CANChannel);
      this->local_transmit(tpara.data);
      sleep_for(std::chrono::milliseconds(50));
      auto &&para = this->local_recv();
      if (para.has_value()) {
        if (para->size() >= 4) { // 通讯异常时会读到 0 长度数组
          motion_finish =
              (TARGET_REACH_STATUS::TARGET_REACH == para.value().at(3));
        }
        // 检查周期50+50ms
        sleep_for(std::chrono::milliseconds(50));
      } else {
        RCLCPP_WARN(logger, "wait_motion_finish: communication failure");
        return false;
      }
    } while (!motion_finish);
    RCLCPP_INFO(logger, "motion finish");
    return true;
  }();
  return motion_ret;
}

bool zeroerr_motor_can::local_transmit(const motor_item_can &param) {
  std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 发送前延时

  constexpr size_t FRAME_COUNT = 1;
  CAN_OBJ frame;
  memset(&frame, 0, sizeof(frame));
  frame.ID = this->transmit_frame_id;
  frame.DataLen = param.size();
  frame.SendType = 0;
  frame.RemoteFlag = 0;
  frame.ExternFlag = 0;
  memcpy(frame.Data, param.data(), frame.DataLen);
  if (Transmit(can_info.DeviceType, can_info.DeviceInd, can_info.CANChannel,
               &frame, FRAME_COUNT) != FRAME_COUNT) {
    std::cout << "发送数据异常，返回数量不匹配" << std::endl;
    return false;
  } else {
    // RCLCPP_INFO(logger, "发送数据成功");
    return true;
  }
}

std::optional<std::vector<uint8_t>> zeroerr_motor_can::local_recv() {
  // TODO: 获取缓冲区内容长度

  std::vector<uint8_t> res;
  constexpr UINT WAIT_TIME_MS = 200;
  const size_t MAX_FRAME_COUNT = 1;
  std::vector<CAN_OBJ> frames(MAX_FRAME_COUNT);
  const auto FRAMES_SIZE = sizeof(CAN_OBJ) * frames.size();
  memset(frames.data(), 0, FRAMES_SIZE);
  // 返回帧数量
  auto receive_ret =
      Receive(can_info.DeviceType, can_info.DeviceInd, can_info.CANChannel,
              frames.data(), MAX_FRAME_COUNT, WAIT_TIME_MS);
  RCLCPP_INFO(logger, "收到帧数量:%d", receive_ret);
  if (receive_ret == 0xFFFF'FFFF) {
    std::cout << "读取数据失败" << std::endl;
    return std::nullopt;
  } else if (receive_ret > 0) {
    for (auto i = 0; i < frames[0].DataLen; ++i) {
      res.push_back(frames[0].Data[i]);
    }
    return {std::move(res)};
  } else {
    return std::nullopt;
  }
}
