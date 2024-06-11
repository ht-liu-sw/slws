#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>

enum class ADDRESS_MAP_ITEM_TYPE { FLOAT = 1, UNKNOW = -1 };
enum class ADDRESS_MAP_ITEM_TYPE_LENGTH { FLOAT = sizeof(float), UNKNOW = -1 };
using AMIT = ADDRESS_MAP_ITEM_TYPE;
using AMITL = ADDRESS_MAP_ITEM_TYPE_LENGTH;

using address_map_item_t = struct address_map_item_t {
  const std::string &item_name;
  const AMIT type;
  const AMITL type_length;
  const int32_t item_count;
  address_map_item_t(const std::string &item_name, const AMIT type,
                     const uint32_t item_count)
      : item_name(item_name), type(type),
        type_length((type == AMIT::FLOAT) ? AMITL::FLOAT : AMITL::UNKNOW),
        item_count(item_count) {}
};
/**
 * @brief 解析或生成 地址数据映射列表
 *
 */
class address_mapper {
public:
  /**
   * @brief Construct a new address mapper object
   *
   * @param address_map_str JSON格式字符串，表示地址映射
   */
  address_mapper(const std::string &address_map_str)
      : address_map_str(address_map_str) {
        init();
      }

  auto get_str() { return address_map_str; }

  /**
   * @brief 获取数据帧长度（uint8_t单位）
   *
   * @return size_t
   */
  auto get_frame_size_uint8() -> size_t {
    return frame_size_uint8_t / sizeof(uint8_t);
  }

  auto get_frame_size_uint16() -> size_t {
    return frame_size_uint8_t / sizeof(uint16_t);
  }

  auto get_frame_size_float32() -> size_t{
    return frame_size_uint8_t / sizeof(float);
  }

private:
  void init() {
    // parse data map
    [this]() {
      auto jo = nlohmann::json::parse(address_map_str);
      for (auto &item : jo["ordered_map"]) {
        auto name = item["name"];
        auto channel_num = item["channel_num"];
        std::string type_str = item["type"];
        AMIT type = ADDRESS_MAP_ITEM_TYPE::UNKNOW;
        if (type_str == "float") {
          type = ADDRESS_MAP_ITEM_TYPE::FLOAT;
        }
        item_list.push_back(address_map_item_t(name, type, channel_num));
      }
    }();

    // 初始化帧长度
    frame_size_uint8_t = 0;
    for (auto item : item_list) {
      frame_size_uint8_t += int32_t(item.type_length) * item.item_count;
    }
  }

private:
  std::vector<address_map_item_t> item_list;
  const std::string &address_map_str;
  size_t frame_size_uint8_t = 0;
};