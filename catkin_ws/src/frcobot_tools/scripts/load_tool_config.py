#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
工具配置加载器 - 用于在launch文件中加载工具配置
优先从工具包加载，如果不存在则从旧的tools.yaml加载
"""

import os
import yaml
import rospkg
import sys

def load_tool_config(tool_name):
    """加载工具配置"""
    rospack = rospkg.RosPack()
    
    # 首先尝试从工具包加载
    try:
        tools_path = rospack.get_path('frcobot_tools')
        tool_config_path = os.path.join(tools_path, 'tools', tool_name, 'config', 'tool.yaml')
        
        if os.path.exists(tool_config_path):
            with open(tool_config_path, 'r') as f:
                config = yaml.safe_load(f)
            return config, 'package'
    except (rospkg.ResourceNotFound, IOError):
        pass
    
    # 如果工具包不存在，尝试从旧配置加载
    try:
        desc_path = rospack.get_path('frcobot_description')
        old_tools_path = os.path.join(desc_path, 'config', 'tools.yaml')
        
        if os.path.exists(old_tools_path):
            with open(old_tools_path, 'r') as f:
                old_tools = yaml.safe_load(f)
            
            if tool_name in old_tools.get('tools', {}):
                return old_tools['tools'][tool_name], 'legacy'
    except (rospkg.ResourceNotFound, IOError, KeyError):
        pass
    
    # 如果都不存在，返回none配置
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

