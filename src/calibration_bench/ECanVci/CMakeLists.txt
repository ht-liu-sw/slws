cmake_minimum_required(VERSION 3.10)

# 设置头文件路径变量
set(ECanVci_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}")

# 设置库文件路径变量
set(ECanVci_LIB_DIRS "${CMAKE_CURRENT_LIST_DIR}")

# 设置库文件名变量
#set(AINSTEC_LIBS )
find_library(
    ECanVci_LIBS
    "libECanVci.so.1"
    ECanVci_LIB_DIRS
    NO_DEFAULT_PATH
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB1 REQUIRED libusb-1.0)
# 将这些变量导出，以便其他CMake脚本可以使用
set(ECanVci_LIBS "libECanVci.so.1")
set(libusb_LIBS "libusb.so")