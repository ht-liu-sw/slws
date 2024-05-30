#pragma once
#include "motor_template.h"
#include <algorithm>
#include <asm-generic/errno-base.h>
#include <bits/types/error_t.h>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <cstddef>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <bits/types/struct_timeval.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

using can_frame_t = struct can_frame;
using timeval_t = struct timeval;

using err_info_t = struct err_info {
  int err_no = 0;
  std::string err_msg;
};

class can_comm {
  int fd = -1;
  struct sockaddr_can addr {};
  struct ifreq ifr {};

public:
  can_comm(){};
  ~can_comm() { close(fd); }

public:
  /*/**
   * @brief 初始化连接，设置接收超时时间
   *
   * @param if_name CAN设备名称, e.g. "can0"
   * @param recv_timeout 超时时间 默认500ms
   * @return bool 连接是否成功
   */
  auto init(const std::string &if_name,
            const timeval_t recv_timeout = {.tv_sec = 0, .tv_usec = 500'000})
      -> bool {

    auto ifname = if_name.c_str();

    if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      perror("Error while opening socket");
      return false;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(fd, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout,
               sizeof(recv_timeout));

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      perror("Error in socket bind");
      return false;
    }
    return true;
  }

  /**
   * @brief 发送 can 帧，返回写的长度
   *
   * @param frame
   * @return auto
   */
  auto send_frame(const can_frame_t &frame) -> size_t {
    const auto nbytes = write(fd, &frame, sizeof(can_frame_t));
    if (nbytes < 0) {
      perror("write can frame");
    }
    return nbytes;
  }

  using read_frame_result_t =
      std::tuple<std::optional<can_frame_t>, err_info_t>;
  /**
   * @brief 读取数据，读取失败时返回 std::nullopt
   *
   * @return std::optional<can_frame_t>
   */
  auto read_frame(const bool need_print_error = true) -> read_frame_result_t {
    can_frame_t frame{};
    err_info_t info;
    const auto nbytes = read(fd, &frame, sizeof(can_frame_t));

    if (nbytes < 0) {
      if (need_print_error == true)
        perror("recv a frame");
      return read_frame_result_t{std::nullopt, std::move(info)};
    }
    return read_frame_result_t{std::move(frame), std::move(info)};
  }

  /**
   * @brief 生成 一个can_frane_t 数据
   *
   * @param can_id
   * @param can_dlc
   * @param data
   * @return auto
   */
  static auto dump_can_frame(const uint32_t can_id, const can_data_t data) {
    can_frame_t frame;
    frame.can_id = can_id;
    frame.can_dlc = data.size();
    memset(frame.data, 0, sizeof(frame.data));
    std::memcpy(frame.data, data.data(), data.size());
    return frame;
  }

  // bool clear_input_buffer() {
  //   int nbytes = ioctl(fd, SIOCOUTQ, &ifr);
  //   if (nbytes != 0) {
  //     printf("Error: Failed to clear the buffer.\n");
  //   }
  // }
  // bool clear_output_buffer();
  //
  // bool clear_both_buffer();
};

// void example(void) {
//   can_comm handle;
//   handle.init("can0");
//   can_frame_t frame{.can_id = 0x643, .can_dlc = 2, .data = {0x00, 0x02}};
//   handle.send_frame(frame);
//   const auto recv_frame = handle.read_frame();
//   if (recv_frame.has_value()) {
//     printf("%x %d <%x,%x,%x,%x,%x,%x,%x,%x>\n", recv_frame->can_id,
//            recv_frame->can_dlc, recv_frame->data[0], recv_frame->data[1],
//            recv_frame->data[2], recv_frame->data[3], recv_frame->data[4],
//            recv_frame->data[5], recv_frame->data[6], recv_frame->data[7]);
//   }
// }

// int main(){
//   example();
// }
