# SLWS
力标定控制软件集合，主要包括以下模块：
## 中央控制模块`central_control`
## ABB机械臂控制模块`abb_ros2`
控制原理：
1. 靠模块远程更新运动参数
2. 机械臂持续运行RAPID程序，不断读取运动参数，触发后机械臂开始运动
## 标定台控制模块`calibration_bench`
4路双电机CAN通讯控制：
### CANID编号规律:
- XZ电机CANID = 标定台号 * 2 - 1
- XY电机CANID = 标定台号 * 2 - 0
- 示例: 
  - 2号标定台,XZ电机->3
  - 3号标定台,XZ电机->5
  - 3号标定台,XY点击->6
  - 4号标定台,XY点击->8
## 传感器数据处理模块`sensor_process`
## 面板`panel`