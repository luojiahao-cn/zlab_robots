# 机械臂命令消息
string command_type    # 命令类型，例如: "move_j", "move_l", "grip", "set_speed", 等
string command_data    # 命令数据，可以是JSON格式字符串以支持复杂参数
int32 command_id       # 命令ID，用于追踪命令执行