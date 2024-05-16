cmake_minimum_required(VERSION 3.8)
project(calibration_bench)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lusb)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sl_interfaces REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/ECanVci/CMakeLists.txt)
include_directories(${ECanVci_INCLUDE_DIRS})
link_directories(${ECanVci_LIB_DIRS})
install(FILES ${ECanVci_LIB_DIRS}/${ECanVci_LIBS}
  DESTINATION lib/)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib/)

add_executable(${PROJECT_NAME} 
  src/calibration_bench.cpp
  src/motor_template.cpp
  src/zeroerr_motor_can.cpp
  src/main.cpp
  )
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty>
  $<INSTALL_INTERFACE:include>)

target_include_directories(${PROJECT_NAME} PUBLIC ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  ${ECanVci_LIB_DIRS}/${ECanVci_LIBS}
  ${ECanVci_LIB_DIRS}/${libusb_LIBS}
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17 -> C++20
ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "sl_interfaces"
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)

  add_executable(test_can
    src/calibration_bench.cpp
    src/motor_template.cpp
    src/zeroerr_motor_can.cpp
    test/test.cpp
    )
  target_include_directories(test_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty>
    $<INSTALL_INTERFACE:include>)

  target_include_directories(test_can PUBLIC ${LIBUSB_INCLUDE_DIRS})
  target_link_libraries(
    test_can
    ${ECanVci_LIB_DIRS}/${ECanVci_LIBS}
    ${ECanVci_LIB_DIRS}/${libusb_LIBS}
  )
  target_compile_features(test_can PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17 -> C++20
  ament_target_dependencies(
    test_can
    "rclcpp"
    "sl_interfaces"
  )
  install(TARGETS test_can
  DESTINATION lib/test_can)

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
