import rosbag
import pandas as pd

bag = rosbag.Bag('catkin_ws/data/magnetic_data/2025-04-15-21-10-34.bag')
data = []



for topic, msg, t in bag.read_messages(topics=['/tf']):
    for transform in msg.transforms:
        if transform.child_frame_id == 'magnet_predicted':
            trans = transform.transform.translation
            rot = transform.transform.rotation
            data.append([
                t.to_sec(),
                trans.x, trans.y, trans.z,
                rot.x, rot.y, rot.z, rot.w
            ])

df = pd.DataFrame(data, columns=['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
df.to_csv('catkin_ws/data/magnetic_data/tf_history.csv', index=False)