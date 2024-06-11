#pragma once

#include <optional>
#include <vector>

class sensor_conn {
public:
  sensor_conn(){};
  virtual ~sensor_conn(){};
  virtual auto init() -> bool = 0;
  virtual auto exit() -> bool = 0;
  virtual auto capture_once() -> std::optional<std::vector<double>> = 0;
};