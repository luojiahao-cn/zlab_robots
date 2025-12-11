#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
工具配置加载器 - 用于在launch文件中加载工具配置
从工具包加载配置
"""

import os
import yaml
import rospkg
import sys

def load_tool_config(tool_name):
    """加载工具配置"""
    rospack = rospkg.RosPack()
    
    # 从工具包加载
    try:
        tools_path = rospack.get_path('frcobot_tools')
        tool_config_path = os.path.join(tools_path, 'tools', tool_name, 'config', 'tool.yaml')
        
        if os.path.exists(tool_config_path):
            with open(tool_config_path, 'r') as f:
                config = yaml.safe_load(f)
            return config, 'package'
    except (rospkg.ResourceNotFound, IOError):
        pass
    
    # 如果工具包不存在，返回none配置
    return {
        'tool_type': 'simple',
        'origin': {'xyz': [0,0,0], 'rpy': [0,0,0]},
        'tcp': {'xyz': [0,0,0], 'rpy': [0,0,0]},
        'physical_properties': {
            'mass': 0.1,
            'inertia': {'ixx': 0.1, 'ixy': 0.0, 'ixz': 0.0, 
                       'iyy': 0.1, 'iyz': 0.0, 'izz': 0.1}
        }
    }, 'none'

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: load_tool_config.py <tool_name>")
        sys.exit(1)
    
    tool_name = sys.argv[1]
    config, source = load_tool_config(tool_name)
    
    # 输出YAML格式的配置（用于xacro）
    print(yaml.dump(config, default_flow_style=False))

