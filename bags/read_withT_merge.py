import pandas as pd
import matplotlib.pyplot as plt
import struct
import rosbag2_py
import numpy as np

def create_transformation_matrix(R, p):
    T = np.eye(4)  # 初始化为单位矩阵
    T[:3, :3] = R  # 设置旋转部分
    T[:3, 3] = p   # 设置平移部分
    return T

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    r00 = 1 - 2 * (qy**2 + qz**2)
    r01 = 2 * (qx * qy - qz * qw)
    r02 = 2 * (qx * qz + qy * qw)

    r10 = 2 * (qx * qy + qz * qw)
    r11 = 1 - 2 * (qx**2 + qz**2)
    r12 = 2 * (qy * qz - qx * qw)

    r20 = 2 * (qx * qz - qy * qw)
    r21 = 2 * (qy * qz + qx * qw)
    r22 = 1 - 2 * (qx**2 + qy**2)

    return np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])

def invert_transformation_matrix(T):
    R = T[:3, :3]
    p = T[:3, 3]

    # 计算逆矩阵
    R_inv = R.T  # 旋转矩阵的逆是转置
    p_inv = -R_inv @ p  # 平移部分

    # 构造逆变换矩阵
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = p_inv

    return T_inv

# 读取ROSbag数据
def read_rosbag2(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    data1, data2 = [], []

    while reader.has_next():
        topic, serialized_msg, timestamp = reader.read_next()
        
        if topic == '/fused_position':
            x, y, z = struct.unpack('ddd', serialized_msg[4:28])
            time_in_sec = timestamp / 1e9
            data1.append((time_in_sec, x, z))
            
        elif topic == '/reflict_center':
            x, y, z = struct.unpack('ddd', serialized_msg[4:28])
            time_in_sec = timestamp / 1e9
            data2.append((time_in_sec, x, z))

    df1 = pd.DataFrame(data1, columns=['time', 'x', 'z'])
    df2 = pd.DataFrame(data2, columns=['time', 'x', 'z'])
    return df1, df2

# 读取CSV文件数据
file_path = 'IR_018.csv'
column_names = [
    "Frame", "Time (Seconds)",
    "RigidBody003_RotX", "RigidBody003_RotY", "RigidBody003_RotZ", "RigidBody003_RotW",
    "RigidBody003_PosX", "RigidBody003_PosY", "RigidBody003_PosZ",
    "RigidBody002_RotX", "RigidBody002_RotY", "RigidBody002_RotZ", "RigidBody002_RotW",
    "RigidBody002_PosX", "RigidBody002_PosY", "RigidBody002_PosZ"
]
df = pd.read_csv(file_path, skiprows=7, names=column_names)

positions_in_rigid003_frame = []

for _, row in df.iterrows():
    rigid003_qx, rigid003_qy, rigid003_qz, rigid003_qw = row["RigidBody003_RotX"], row["RigidBody003_RotY"], row["RigidBody003_RotZ"], row["RigidBody003_RotW"]
    rigid002_qx, rigid002_qy, rigid002_qz, rigid002_qw = row["RigidBody002_RotX"], row["RigidBody002_RotY"], row["RigidBody002_RotZ"], row["RigidBody002_RotW"]
    rigid003_p = np.array([row["RigidBody003_PosX"], row["RigidBody003_PosY"], row["RigidBody003_PosZ"]])
    rigid002_p = np.array([row["RigidBody002_PosX"], row["RigidBody002_PosY"], row["RigidBody002_PosZ"]])

    # 计算 RigidBody003 的旋转矩阵
    R03 = quaternion_to_rotation_matrix(rigid003_qx, rigid003_qy, rigid003_qz, rigid003_qw)
    R02 = quaternion_to_rotation_matrix(rigid002_qx, rigid002_qy, rigid002_qz, rigid002_qw)

    T03 = create_transformation_matrix(R03, rigid003_p.T)
    T02 = create_transformation_matrix(R02, rigid002_p.T)

    T32 = invert_transformation_matrix(T03) @ T02

    T23 = invert_transformation_matrix(T32)

    rigid002_position_in_rigid003 = T23[:3, 3]

    positions_in_rigid003_frame.append(rigid002_position_in_rigid003)

positions_df = pd.DataFrame(positions_in_rigid003_frame, columns=["x_in_rigid003", "y_in_rigid003", "z_in_rigid003"])
bag_path = '/home/gxh/Desktop/bags/rosbag2_2024_11_26-23_53_03'
df1, df2 = read_rosbag2(bag_path)

plt.figure(figsize=(10, 10))
print(positions_df)
plt.plot(-positions_df["x_in_rigid003"], positions_df["z_in_rigid003"], label="RigidBody002 in RigidBody003 Frame", color="green", marker=".")
plt.scatter(df1["x"], df1["z"], marker='o', color='blue', label='Fused Position')
plt.scatter(df2["x"], df2["z"], marker='x', color='orange', label='Reflect Center')

plt.xlabel("X Position (meters)")
plt.ylabel("Y Position (meters)")
plt.title("Relative Position and ROSbag Trajectories")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.xlim(-5, 5)
plt.ylim(0, 5)
plt.show()
