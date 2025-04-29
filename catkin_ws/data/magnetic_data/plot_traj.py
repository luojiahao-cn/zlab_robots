import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# 选择功能模式: 1=单文件绘图  2=批量分别绘图  3=批量合并绘图
mode = 3  # 修改为1、2或3选择功能

# 每个文件对应的读取行数
file_n_map = {
    'tf_record_1.010.csv': 2500,
    'tf_record_1.020.csv': 2000,
    'tf_record_1.030.csv': 1400,
    'tf_record_1.040.csv': 1000,
    'tf_record_1.050.csv': 600,
    'tf_record_1.060.csv': 600,
    'tf_record_1.070.csv': 500,
    'tf_record_1.080.csv': 300,
    'tf_record_1.090.csv': 100,
}

if mode == 1:
    # 单文件绘图
    csv_file = 'catkin_ws/data/magnetic_data/tf_record_1.090.csv'
    N = file_n_map.get(os.path.basename(csv_file), 100)
    df = pd.read_csv(csv_file)
    df = df[df['frame'] == 'magnet_predicted']
    df = df.head(N)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(df['x'], df['y'], df['z'], label='Magnet Trajectory')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Magnet 3D Trajectory')
    ax.legend()
    plt.show()

elif mode == 2:
    # 批量分别绘图
    csv_files = glob.glob('catkin_ws/data/magnetic_data/tf_record_*.csv')
    for csv_file in csv_files:
        base = os.path.basename(csv_file)
        N = file_n_map.get(base, 100)
        df = pd.read_csv(csv_file)
        df = df[df['frame'] == 'magnet_predicted']
        df = df.head(N)
        if df.empty:
            continue
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(df['x'], df['y'], df['z'], label='Magnet Trajectory')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"Magnet 3D Trajectory\n{base}")
        ax.legend()
        out_name = os.path.splitext(base)[0] + '.png'
        plt.savefig(out_name, dpi=300)  # 提高图片分辨率
        plt.close(fig)

elif mode == 3:
    # 批量合并绘图
    csv_files = glob.glob('catkin_ws/data/magnetic_data/tf_record_*.csv')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    colors = plt.cm.get_cmap('tab10', len(csv_files))
    for idx, csv_file in enumerate(csv_files):
        base = os.path.basename(csv_file)
        N = file_n_map.get(base, 100)
        df = pd.read_csv(csv_file)
        df = df[df['frame'] == 'magnet_predicted']
        df = df.head(N)
        if df.empty:
            continue
        ax.plot(df['x'], df['y'], df['z'],
        label=base,
        color=colors(idx),
        linewidth=0.8)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Magnet 3D Trajectory (All Files)')
    # ax.legend()
    plt.tight_layout()
    plt.savefig('all_magnet_traj.png', dpi=1000)  # 提高图片分辨率
    plt.show()
else:
    print("mode 只能为 1、2 或 3")