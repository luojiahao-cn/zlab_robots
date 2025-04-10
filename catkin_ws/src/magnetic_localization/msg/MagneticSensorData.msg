std_msgs/Header header
uint32 sensor_id                        # 传感器ID
geometry_msgs/PoseStamped sensor_pose   # 传感器位置和方向
geometry_msgs/Vector3 field             # 磁场强度向量
bool is_valid                           # 数据有效性标志