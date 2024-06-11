# Sensor Process
## 概述
### 本节点专门处理传感器数据，设计如下:
1. 对单个传感器（modbus设备）持续缓存传感器数据，提供开始、结束和清除接口（对应整个标定流程）
    - 开始接口`name`
        - 输入参数：
            1. `tag`:文本标识，人工输入的，用于用户分辨数据
            2. `sensor_type_tag`:文本标识，标识传感器类型，仅用于查看
            3. `address_map`:地址映射字符串（顺序的JSON格式字符串，连续），影响数据保存
        - 返回参数:
            - `success`:是否成功，为`false`则`uuid`无效
            - `uuid`:该份数据的唯一标识，用于后续处理
        - 地址映射字符串示例：
             ```json
             {  // PhotonR56P系列
               "ordered_map": [          // 顺序的数据映射数组，从0地址开始
                 {
                   "name": "adc_original",   // 字段标记名称，该字段为首个字段，从 0 地址       开始
                   "channel_num": 8,         // 该字段通道数量
                   "type": "float"           // 字段类型，该字段总长度(modbus长度单位)为    sizeof(float) /      sizeof(uint16_t) * channel_num
                 },
                 {
                   "name": "temp",           // 同上，且该字段地址紧接着上个字段
                   "channel_num": 4,
                   "type": "float"
                 },
                 {
                   "name": "adc_processed", // 同上
                   "channel_num": 8,
                   "type": "float"
                 },
                 {
                   "name": "force",
                   "channel_num": 3,
                   "type": "float"
                 },
                 {
                   "name": "torque",
                   "channel_num": 3,
                   "type": "float"
                 }
               ]
             }
             ``` 
    - 结束接口`name`
      - 结束后该数据将停留在缓存中，等待处理
    - 清除接口`remove`
      - 输入`uuid`，尝试从缓存中去除该数据
      - 输出`success`，找不到数据则返回`false`，否则清除成功
2. 对单个传感器，记录从**单步开始时间**到**单步结束时间**的时间戳以及输入的**字符串标记**（对应单步数据），影响数据保存
    - 输入
      - `uuid`
      - 字符串标记
      - 开始时间
      - 停止时间
3. 获取缓存数据列表，能获取当前缓存的全部数据的粗略信息
    - 输出
        - `tag`: 文本标识，人工输入的，用于用户分辨数据
        - `sensor_type_tag`: 文本标识，标识传感器类型，仅用于查看
        - `address_map`: 地址映射字符串（顺序的JSON格式字符串，连续），影响数保存
        - `start_time`: 时间戳标识，表示开始时间
        - `stop_time`: 时间戳标识，表示停止时间
        - `uuid`: 唯一标记
4. 获取缓存数据
   - 输入
        - `uuid`
   - 输出
        - `tag`: 文本标识，人工输入的，用于用户分辨数据
        - `sensor_type_tag`: 文本标识，标识传感器类型，仅用于查看
        - `address_map`: 地址映射字符串（顺序的JSON格式字符串，连续），影响数保存
        - `start_time`: 时间戳标识，表示开始时间
        - `stop_time`: 时间戳标识，表示停止时间
        - `uuid`: 唯一标记
        - `sub_cap_list`: 列表<字符串标识，开始时间，结束时间>
        - `data`: 输出数据，借用sensor_msgs/PointCloud2类型进行存储