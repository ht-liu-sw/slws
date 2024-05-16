#include "motor_template.h"
#include <chrono>
#include <stdexcept>
#include <thread>

bool motor_template::is_inited(){
  return init_finish.load();
}

