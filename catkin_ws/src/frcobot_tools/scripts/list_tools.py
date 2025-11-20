#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
列出所有可用的工具包
"""

import os
import yaml
import rospkg

def find_tools():
    """查找所有工具包"""
    rospack = rospkg.RosPack()
    try:
        tools_path = rospack.get_path('frcobot_tools')
        tools_dir = os.path.join(tools_path, 'tools')
    except rospkg.ResourceNotFound:
        # 如果包未找到，尝试使用相对路径
        script_dir = os.path.dirname(os.path.abspath(__file__))
        tools_dir = os.path.join(script_dir, '..', 'tools')
        tools_dir = os.path.abspath(tools_dir)
    
    if not os.path.exists(tools_dir):
        print("错误: 找不到工具目录: {}".format(tools_dir))
        return []
    
    tools = []
    for item in os.listdir(tools_dir):
        tool_path = os.path.join(tools_dir, item)
        if os.path.isdir(tool_path) and not item.startswith('.'):
            config_file = os.path.join(tool_path, 'config', 'tool.yaml')
            if os.path.exists(config_file):
                try:
                    with open(config_file, 'r') as f:
                        config = yaml.safe_load(f)
                    tool_info = config.get('tool_info', {})
                    tools.append({
                        'name': item,
                        'display_name': tool_info.get('display_name', item),
                        'description': tool_info.get('description', ''),
                        'type': config.get('tool_type', 'unknown')
                    })
                except Exception as e:
                    print("警告: 无法读取工具 {} 的配置: {}".format(item, e))
    
    return tools

def main():
    """主函数"""
    tools = find_tools()
    
    if not tools:
        print("未找到任何工具包")
        return
    
    print("\n可用工具包列表:")
    print("=" * 80)
    print("{:<20} {:<30} {:<15} {}".format("工具名称", "显示名称", "类型", "描述"))
    print("-" * 80)
    
    for tool in sorted(tools, key=lambda x: x['name']):
        desc = tool['description'][:30] + "..." if len(tool['description']) > 30 else tool['description']
        print("{:<20} {:<30} {:<15} {}".format(
            tool['name'],
            tool['display_name'],
            tool['type'],
            desc
        ))
    
    print("=" * 80)
    print("\n使用方法:")
    print("  roslaunch <package> <launch_file> tool_name:=<工具名称>")
    print("\n示例:")
    print("  roslaunch fr5v6_single_moveit_config demo.launch tool_name:=tool_250mm")

if __name__ == '__main__':
    main()

