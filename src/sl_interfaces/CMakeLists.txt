cmake_minimum_required(VERSION 3.8)
project(sl_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  #"msg/Num.msg"
  "srv/MotorXYXZPositionControl.srv"
  "srv/UpdateMotorXYXZOffset.srv"
  # --- msg ---
  # --- about package `sensor_process` ---
  "sensor_process/SensorInfo.msg"
  "sensor_process/SensorHeader.msg"
  "sensor_process/SensorRecordData.msg"
  "sensor_process/SensorRecordStepInfo.msg"
  "sensor_process/SensorRecordSingleFrame.msg"
  "sensor_process/StartNewSensorRecorder.srv"
  "sensor_process/GetSensorRecorderData.srv"
  "sensor_process/ReleaseSensorRecorder.srv"
  "sensor_process/FinishRecorder.srv"
  "sensor_process/SensorInsertStepInfo.srv"
  DEPENDENCIES 
  std_msgs 
  sensor_msgs 
  builtin_interfaces
 )

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
