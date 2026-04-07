import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

def invert_transform(x, y, z, roll, pitch, yaw, parent_frame, child_frame):
    # 1. 构造旋转矩阵 (Euler角顺序为 extrinsic XYZ, 即 R=Rz*Ry*Rx)
    # ROS 2 static_transform_publisher 默认使用的是这种顺序
    rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    rot_matrix = rotation.as_matrix()

    # 2. 构造 4x4 齐次变换矩阵 T
    T = np.eye(4)
    T[0:3, 0:3] = rot_matrix
    T[0:3, 3] = [x, y, z]

    # 3. 计算逆矩阵 T_inv
    # 逆矩阵公式：R_inv = R^T, t_inv = -R^T * t
    T_inv = np.linalg.inv(T)

    # 4. 从逆矩阵中提取平移和旋转 (Euler角)
    inv_x, inv_y, inv_z = T_inv[0:3, 3]
    inv_rot_matrix = T_inv[0:3, 0:3]
    inv_rotation = R.from_matrix(inv_rot_matrix)
    inv_roll, inv_pitch, inv_yaw = inv_rotation.as_euler('xyz', degrees=False)

    # 5. 格式化输出
    print(f"\n--- 原始变换 ({parent_frame} -> {child_frame}) ---")
    print(f"Pos: [{x}, {y}, {z}], RPY: [{roll}, {pitch}, {yaw}]")
    
    print(f"\n--- 反向变换 ({child_frame} -> {parent_frame}) ---")
    print(f"x: {inv_x:.6f}")
    print(f"y: {inv_y:.6f}")
    print(f"z: {inv_z:.6f}")
    print(f"roll: {inv_roll:.6f}")
    print(f"pitch: {inv_pitch:.6f}")
    print(f"yaw: {inv_yaw:.6f}")

    print("\n--- 直接运行以下 ROS 2 命令 ---")
    cmd = (f"ros2 run tf2_ros static_transform_publisher "
           f"--x {inv_x:.6f} --y {inv_y:.6f} --z {inv_z:.6f} "
           f"--roll {inv_roll:.6f} --pitch {inv_pitch:.6f} --yaw {inv_yaw:.6f} "
           f"--frame-id {child_frame} --child-frame-id {parent_frame}")
    print(cmd + "\n")

if __name__ == "__main__":
    if len(sys.argv) != 9:
        print("用法: python3 tf_invert.py <x> <y> <z> <roll> <pitch> <yaw> <origin_frame> <origin_child_frame>")
        print("示例: python3 tf_invert.py 1.0 0.0 0.0 0.0 0.0 1.57 world camera")
        sys.exit(1)

    try:
        args = [float(a) for a in sys.argv[1:7]]
        frames = sys.argv[7:9]
        invert_transform(*args, *frames)
    except ValueError:
        print("错误: x, y, z, r, p, y 必须是数字")