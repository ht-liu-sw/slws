builtin_interfaces/Time start_time      # 本次记录的开始时间
builtin_interfaces/Time finish_time     # 本次记录的结束时间
SensorRecordStepInfo[] step_info        # 单步数据时间戳
int32 id                               # 本组数据的id，由sensor_recorder生成
SensorRecordSingleFrame[] data_frames   # 存储 单次记录的数据