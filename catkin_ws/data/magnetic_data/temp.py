import rosbag
import pandas as pd

bag = rosbag.Bag('catkin_ws/data/magnetic_data/tf_record_1.090.bag')
data = []

# 需要输出的frame列表
frames_to_save = ['magnet_predicted', 'frrobot_j6_link' ]  # 可按需添加

for topic, msg, t in bag.read_messages(topics=['/tf']):
    for transform in msg.transforms:
        if transform.child_frame_id in frames_to_save:
            trans = transform.transform.translation
            rot = transform.transform.rotation
            data.append([
                t.to_sec(),
                transform.child_frame_id,  # 增加frame信息
                trans.x, trans.y, trans.z,
                rot.x, rot.y, rot.z, rot.w
            ])

df = pd.DataFrame(data, columns=['time', 'frame', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
df.to_csv('catkin_ws/data/magnetic_data/tf_record_1.090.csv', index=False)

