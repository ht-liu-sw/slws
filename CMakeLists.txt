cmake_minimum_required(VERSION 3.8)
project(sensor_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
#find libmodbus
include_directories(/usr/include/modbus)

add_executable(sensor_process_node
  src/sensor_conn/sensor_conn_modbus_tcp.cpp
  src/sensor_process/sensor_data_record.cpp
  src/sensor_process_node.cpp
  src/main.cpp)
target_link_libraries(sensor_process_node modbus)
target_include_directories(sensor_process_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})
target_compile_features(sensor_process_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
ament_target_dependencies(
  sensor_process_node
  "sl_interfaces"
  "rclcpp"
)
install(TARGETS sensor_process_node
  DESTINATION lib/${PROJECT_NAME})


# test
add_executable(test_node
  test/test_node.cpp)
target_link_libraries(test_node)
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS})
target_compile_features(test_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
ament_target_dependencies(
  test_node
  "sl_interfaces"
  "rclcpp"
)
install(TARGETS test_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(test_recorder
test/test_recorder.cpp
src/sensor_conn/sensor_conn_modbus_tcp.cpp
src/sensor_process/sensor_data_record.cpp
)
target_link_libraries(test_recorder modbus)
target_include_directories(test_recorder PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${Boost_INCLUDE_DIRS})
target_compile_features(test_recorder PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17
ament_target_dependencies(
  test_recorder
"sl_interfaces"
"rclcpp"
)
install(TARGETS test_recorder
DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
