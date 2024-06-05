SensorDataConnectionInfo conn_info         # 通讯信息
SensorDataHeader header                    # 文本标识，传感器类型，数据映射
SensorDataInfo info                        # 标识开始时间、结束时间和uuid
SensorDataStepCapInfo[] list_step_cap_info # 表示单步数据的列表
SensorDataFrame[] data_array               # 存放数据