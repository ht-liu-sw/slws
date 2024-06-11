# MODBUS_TCP
string  modbus_tcp_ip                       # modbus_tcp ip
uint32  modbus_tcp_port                     # modbus_tcp 端口号
uint8   modbus_tcp_slave_id                 # 从站id
string  tag                                 # 文本标识，人工输入的，用于用户分辨数据
string  sensor_type_tag                     # 文本标识，标识传感器类型，仅用于查看
string  address_map                         # 地址映射字符串（顺序的JSON格式字符串，连续），影响数据保存