import yaml
import math
import os
from copy import deepcopy
from scipy.spatial.transform import Rotation as R

class TagPoseTransformer:
    def __init__(self, rotation_angle_deg=90):
        """
        初始化变幻器
        :param rotation_angle_deg: 顺时针旋转的角度。按照用户需求，基准坐标系顺时针旋转90度，
                                  相当于标签在新的坐标系下逆时针旋转90度 (+pi/2)。
        """
        self.angle_rad = math.radians(rotation_angle_deg)
        # 关键：这是“左乘”的旋转（坐标系变换）
        self.Rz = R.from_euler('z', self.angle_rad, degrees=False)

    def rotate_pose(self, pose):
        if 'position' not in pose or 'rotation' not in pose:
            return pose

        pose = deepcopy(pose)

        x = pose['position'].get('x', 0.0)
        y = pose['position'].get('y', 0.0)
        z = pose['position'].get('z', 0.0)

        rx = pose['rotation'].get('x', 0.0)
        ry = pose['rotation'].get('y', 0.0)
        rz = pose['rotation'].get('z', 0.0)

        # 1) position: p' = Rz(+90) p  -> (x,y)->(-y,x)
        new_x, new_y, new_z = -y, x, z

        # 2) rotation: R' = Rz(+90) * R_old   (左乘！)
        R_old = R.from_euler('xyz', [rx, ry, rz], degrees=False)
        R_new = self.Rz * R_old
        new_rx, new_ry, new_rz = R_new.as_euler('xyz', degrees=False)

        # 归一化到 [-pi, pi]（scipy 通常已经给你一个等价表示，但保留也行）
        def wrap(a): 
            return math.atan2(math.sin(a), math.cos(a))
        new_rx, new_ry, new_rz = wrap(new_rx), wrap(new_ry), wrap(new_rz)

        pose['position']['x'] = float(new_x)
        pose['position']['y'] = float(new_y)
        pose['position']['z'] = float(new_z)

        pose['rotation']['x'] = float(new_rx)
        pose['rotation']['y'] = float(new_ry)
        pose['rotation']['z'] = float(new_rz)

        # 3) noise: 90°下的“交换”近似可保留，但建议只在原有字段时操作
        if 'position_noise' in pose and isinstance(pose['position_noise'], dict):
            pn = pose['position_noise']
            if 'x' in pn and 'y' in pn:
                pn['x'], pn['y'] = pn['y'], pn['x']

        if 'rotation_noise' in pose and isinstance(pose['rotation_noise'], dict):
            rn = pose['rotation_noise']
            if 'x' in rn and 'y' in rn:
                rn['x'], rn['y'] = rn['y'], rn['x']

        return pose

    def transform_yaml(self, input_file, output_file, target_bodies):
        """
        读取并变幻指定的 body 中的 tag 位姿
        """
        with open(input_file, 'r') as f:
            data = yaml.safe_load(f)

        if 'bodies' not in data:
            print("Error: No 'bodies' found in YAML.")
            return

        for body_entry in data['bodies']:
            # body_entry 是一个字典，键是 body 的名字
            for body_name, body_info in body_entry.items():
                if body_name in target_bodies:
                    print(f"Transforming body: {body_name}")
                    if 'tags' in body_info:
                        for tag in body_info['tags']:
                            if 'pose' in tag:
                                tag['pose'] = self.rotate_pose(tag['pose'])

        with open(output_file, 'w') as f:
            # 使用 flow_style=False 保持层级结构
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        print(f"Successfully saved to {output_file}")

def main():
    # 获取脚本所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    input_yaml = os.path.join(current_dir, 'tagslam.yaml')
    output_yaml = os.path.join(current_dir, 'tagslam_transformed1.yaml')

    # 定义需要处理的标定工具名称
    calibration_tools = [
        'diana7_flange_calib',
        'arm1_flange_calib',
        'arm2_flange_calib'
    ]

    transformer = TagPoseTransformer(rotation_angle_deg=90)
    transformer.transform_yaml(input_yaml, output_yaml, calibration_tools)

if __name__ == "__main__":
    main()
